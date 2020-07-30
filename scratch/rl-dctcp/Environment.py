import numpy as np
import subprocess
from pathlib import Path
from helper import pretty, softmax
from flowmon import parse

WEIGHT = 'Weight.txt'


def vector_to_csv(vector, file_name, action):
    string = ','.join(pretty(_) for _ in vector)
    with open(file_name, action) as file:
        return file.write(string + '\n')

def action_to_file(vector, file_name, action,filename2):
    vector_to_csv(vector,filename2,'a')      
    l1=[]
    l2=[]
    for i in range(0,64,8):
        l1.append(vector[i:i+8])
    for i in range(64,128,8):
        l2.append(vector[i:i+8])
    l1 = np.array(l1)
    l2 = np.array(l2)
    c=np.sum([l1,l2],axis=0)
    ll=[]
    for a in c:
        s= np.sum(a);
        l=[]
        for i in a:
            l.append(i/s)
        ll.append(l)
    for i in range(8):
        for j in range(1,8):
            ll[i][j]=ll[i][j-1]+ll[i][j]
    #print(vector)
    with open(file_name, action) as file:
        for i in ll:
            for j in i:
                 file.write(str(j)+"  ")
            file.write("\n")

# FROM FILE
def file_to_csv(file_name):
    # reads file, outputs csv
    with open(file_name, 'r') as file:
        return file.readline().strip().strip(',')

def csv_to_matrix(string, nodes_num):
    # reads text, outputs matrix
    v = np.asarray(tuple(float(x) for x in string.split(',')[:nodes_num**2]))
    M = np.split(v, nodes_num)
    return np.vstack(M)


# FROM RL
def rl_to_matrix(vector, nodes_num):
    M = np.split(vector, nodes_num)
    for _ in range(nodes_num):
        M[_] = np.insert(M[_], _, -1)
    return np.vstack(M)


# label environment
class LinkweightEnv():

    def __init__(self, DDPG_config, folder):
        self.folder = folder
        self.a_dim = DDPG_config['ACTION_DIM']
        self.s_dim = DDPG_config['STATE_DIM']
        self.env_S = np.full([self.s_dim], 0, dtype=float)           # state
        self.env_A = np.full([self.a_dim], 1, dtype=int)           # weights
        self.env_R = 0.0  # reward
        self.counter = 0

    def reset(self):
        res = subprocess.call('sudo ./waf --run "wcmp" >log.log', stdout=subprocess.PIPE, shell = True)
        parse()
        fname = 'acflow.txt'
        l=[]
        with open(fname, 'r+', encoding='utf-8') as f:
            for i in f.readlines():
                s = i[:-1].split()
                l.extend(s)
        while len(l) < 50:
            l.extend([0,0,0,0,0])
        if len(l) > 50:
           l = l[0:50]
        
        fname = 'finishflow.txt'
        ll=[]
        with open(fname, 'r+', encoding='utf-8') as f:
            for i in f.readlines():
                s = i[:-1].split()
                ll.extend(s)
        while len(ll) < 700:
            ll.extend([0,0,0,0,0,0,0])
        if len(ll) > 700:
            ll = ll[0:700]
        l.extend(ll)
        l = [float(i) for i in l]
        self.env_S = np.array(l)
        return self.env_S

    def step(self, action):
        print(action)
        self.counter += 1
        self.env_A = action
        # write to file input for ns3: link weight
        filename2 = self.folder+ 'actionLog.csv'
        action_to_file(self.env_A, WEIGHT, 'w',filename2)

        #read last throughput
        fname = 'throughput.txt'
        old_th = 0.0
        with open(fname, 'r+', encoding='utf-8') as f:
            content = f.read()
            old_th = float(content) 

        #excute ns3
        res = subprocess.call('sudo ./waf --run "conga-simulation-large --runMode=WCMP" >log.log',shell = True)
        parse()
        # read ns3's output: next state and reward
        fname = 'acflow.txt'
        l=[]
        with open(fname, 'r+', encoding='utf-8') as f:
            for i in f.readlines():
                s = i[:-1].split()
                l.extend(s)
        if len(l) > 50:
           l = l[0:50]
        while len(l) < 50:
            l.extend([0,0,0,0,0])
        fname = 'finishflow.txt'
        ll=[]
        with open(fname, 'r+', encoding='utf-8') as f:
            for i in f.readlines():
                s = i[:-1].split()
                ll.extend(s)
        if len(ll) > 700:
           ll = ll[0:700]
        while len(ll) < 700:
            ll.extend([0,0,0,0,0,0,0])
        l.extend(ll)
        l = [float(i) for i in l]
        self.env_S = np.array(l)

        #read new throughput
        fname = 'throughput.txt'
        new_th = 0.0
        with open(fname, 'r+', encoding='utf-8') as f:
            content = f.read()
            new_th = float(content) 
        self.env_R = new_th/old_th
        # log everything to file
        with open(self.folder +'rewardlog.csv', 'a') as file:
            file.write(str(self.env_R) + '\n')
        
        # return new status and reward
        return self.env_S, self.env_R, 0

    def end(self):
        return
