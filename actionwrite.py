import numpy as np
import random

# 可写函数说明
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
    with open('Weight2.txt', 'w') as file:
        for i in range(8):
            for j in range(8):
                for k in range(8):
                     file.write(str(format(one[i][j][k], '.6f'))+"  ")
                file.write("\n")
    return

a=[]
for i in range(0,128):
    a.append(random.uniform(0,1))
na = np.array(a)

os = action_to_file(na)
print(os)




