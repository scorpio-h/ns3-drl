#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from ns3gym import ns3env
import tensorflow as tf
import sys
import json

from ReplayBuffer import ReplayBuffer
from ActorNetwork import ActorNetwork
from CriticNetwork import CriticNetwork
from OU import OU
from helper import setup_exp, setup_run, parser, pretty, scale
import numpy as np

def vector_to_file(vector, file_name, action):
    string = ','.join( str(i) for i in vector)
    with open(file_name, action) as file:
        return file.write(string + '\n')

def action_to_file(na):
    l1=[]
    l2=[]
    for i in range(0,64,8):
        l1.append(na[i:i+8])

    for i in range(64,128,8):
        l2.append(na[i:i+8])

    l1 = np.array(l1)
    l2 = np.array(l2)

    array = np.zeros((8, 8, 8), dtype=np.float)
    for i in range(8):
        for j in range(8):
            for k in range(8):
                if i == j:
                    array[i,j,k] = 0.1
                else:
                    array[i,j,k] = l1[i][k]+l2[k][j]

    one = np.zeros((8, 8, 8), dtype=np.float)
    for i in range(8):
        for j in range(8):
            c=np.sum(array[i,j])
            for k in range(8):
                one[i,j,k]=array[i,j,k]/c

    for i in range(8):
        for j in range(8):
            for k in range(1,8):
                one[i,j,k] = one[i][j][k-1]+one[i][j][k]
    with open('Weight.txt', 'w') as file:
        for i in range(8):
            for j in range(8):
                for k in range(8):
                     file.write(str(format(one[i][j][k], '.6f'))+"  ")
                file.write("\n")
    return

parser = argparse.ArgumentParser(description='Start simulation script on/off')
parser.add_argument('--start',
                    type=int,
                    default=1,
                    help='Start ns-3 simulation script 0/1, Default: 1')
parser.add_argument('--iterations',
                    type=int,
                    default=1,
                    help='Number of iterations, Default: 1')
args = parser.parse_args()
startSim = bool(args.start)
iterationNum = int(args.iterations)
MAX_STEPS = 10000
stepTime = 0.001 # seconds
simTime = stepTime*MAX_STEPS+0.0001 # seconds
seed = 12
simArgs = {"--EndTime": simTime,"--StepTime" : stepTime,}
debug = False
port = 5555

env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)
env.reset()

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space,  ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

with open('DDPG.json') as jconfig:
    DDPG_config = json.load(jconfig)
DDPG_config['EXPERIMENT'] = setup_exp()
folder = setup_run(DDPG_config)
if DDPG_config['RSEED'] == 0:
    DDPG_config['RSEED'] = None
np.random.seed(DDPG_config['RSEED'])

# Generate an environment

action_dim = DDPG_config['ACTION_DIM']
state_dim = DDPG_config['STATE_DIM']

MU = DDPG_config['MU']
THETA = DDPG_config['THETA']
SIGMA = DDPG_config['SIGMA']

ou = OU(action_dim, MU, THETA, SIGMA)       #Ornstein-Uhlenbeck Process

BUFFER_SIZE = DDPG_config['BUFFER_SIZE']
BATCH_SIZE = DDPG_config['BATCH_SIZE']
GAMMA = DDPG_config['GAMMA']
EXPLORE = DDPG_config['EXPLORE']
EPISODE_COUNT = iterationNum
train_indicator=1

if EXPLORE <= 1:
    EXPLORE = MAX_STEPS * EXPLORE
# SETUP ENDS HERE

epsilon = 1
indicator = 0

#Tensorflow GPU optimization
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
sess = tf.Session(config=config)
from keras import backend as K
K.set_session(sess)

actor = ActorNetwork(sess, DDPG_config)
critic = CriticNetwork(sess, DDPG_config)
buff = ReplayBuffer(BUFFER_SIZE)    #Create replay buffer

ltm = ['a_h0', 'a_h1', 'a_V', 'c_w1', 'c_a1', 'c_h1', 'c_h3', 'c_V']
layers_to_mind = {}
L2 = {}

for k in ltm:
    layers_to_mind[k] = 0
    L2[k] = 0

try:
    actor.model.load_weights( "actormodel.h5")
    critic.model.load_weights("criticmodel.h5")
    actor.target_model.load_weights("actormodel.h5")
    critic.target_model.load_weights("criticmodel.h5")
    print("Weight load successfully")
except:
    print("Cannot find the weight")

#initial state of simulator
loss = 0
stepIdx = 0

try:
    s_t = env.reset()
    s_t = np.array(s_t)
    vector_to_file(s_t , folder + 'stateLog.csv', 'a')
    print("Step: ", stepIdx)
    reward = 0
    done = False
    info = None
    print("---obs: ", s_t)

    while True:
        stepIdx += 1
        epsilon -= 1.0 / EXPLORE

        a_t = np.zeros([1, action_dim])
        noise_t = np.zeros([1, action_dim])
        a_t_original = actor.model.predict(s_t.reshape(1, s_t.shape[0]))

        if train_indicator and epsilon > 0 and (stepIdx % 20) // 100 != 9:
            noise_t[0] = epsilon * ou.evolve()

        a = a_t_original[0]
        n = noise_t[0]
        a_t[0] = np.where((a + n > 0) & (a + n < 1), a + n, a - n).clip(min=0, max=1)

        print("---action: ", a_t[0])

        print("Step: ", stepIdx)
        # execute action
        action_to_file(a_t[0])
        s_t1, r_t, done, info = env.step(a_t[0])
        s_t1 = np.array(s_t1)
        #print("--reward, done: ",r_t,done)
        print("---obs, reward, done, info: ", s_t1, r_t, done, info)

        buff.add(s_t, a_t[0], r_t, s_t1, done)      #Add replay buffer

        scale = lambda x: x
        #Do the batch update
        batch = buff.getBatch(BATCH_SIZE)
        states = scale(np.asarray([e[0] for e in batch]))
        actions = scale(np.asarray([e[1] for e in batch]))
        rewards = scale(np.asarray([e[2] for e in batch]))
        new_states = scale(np.asarray([e[3] for e in batch]))
        dones = np.asarray([e[4] for e in batch])

        y_t = np.zeros([len(batch), action_dim])
        target_q_values = critic.target_model.predict([new_states, actor.target_model.predict(new_states)])

        for k in range(len(batch)):
            if dones[k]:
                y_t[k] = rewards[k]
            else:
                y_t[k] = rewards[k] + GAMMA*target_q_values[k]

        if train_indicator and len(batch) >= BATCH_SIZE:
            loss = critic.model.train_on_batch([states, actions], y_t)
            a_for_grad = actor.model.predict(states)
            grads = critic.gradients(states, a_for_grad)
            # does this give an output like train_on_batch above? NO
            actor.train(states, grads)
            actor.target_train()
            critic.target_train()
            with open(folder + 'lossLog.csv', 'a') as file:
                file.write(pretty(loss) + '\n')

        s_t = s_t1

        for layer in actor.model.layers + critic.model.layers:
            if layer.name in layers_to_mind.keys():
                L2[layer.name] = np.linalg.norm(np.ravel(layer.get_weights()[0])-layers_to_mind[layer.name])
                layers_to_mind[layer.name] = np.ravel(layer.get_weights()[0])

        if train_indicator and len(batch) >= BATCH_SIZE:
            vector_to_file([L2[x] for x in ltm], folder + 'weightsL2' + 'Log.csv', 'a')

        with open(folder + 'rewardLog.csv', 'a') as file:
            file.write(str(r_t) + '\n')
        vector_to_file(a_t[0], folder + 'actionLog.csv', 'a')
        vector_to_file(s_t1 , folder + 'stateLog.csv', 'a')
        vector_to_file(noise_t[0], folder + 'noiseLog.csv', 'a')

        if np.mod((stepIdx+1), 2) == 0:   # writes at every 2nd episode
            if (train_indicator):
                actor.model.save_weights(folder + "actormodel.h5", overwrite=True)
                actor.model.save_weights(folder + "actormodel" + str(stepIdx) + ".h5")
                with open(folder + "actormodel.json", "w") as outfile:
                    outfile.write(actor.model.to_json(indent=4) + '\n')

                critic.model.save_weights(folder + "criticmodel.h5", overwrite=True)
                critic.model.save_weights(folder + "criticmodel" + str(stepIdx) + ".h5")
                with open(folder + "criticmodel.json", "w") as outfile:
                    outfile.write(critic.model.to_json(indent=4) + '\n')

        if done:
            break

except KeyboardInterrupt:
    print("Ctrl-C -> Exit")
finally:
    env.close()
    print("Done")
