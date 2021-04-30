import numpy as np 
import tf 

# ref https://wiki.openstreetmap.org/wiki/JOSM/Plugins/PicLayer#Annexes

M00=0.9437411916931052
M11=0.9437411916931052
M10=0.33068499074145674
M01=-0.33068499074145674
M02=-253.44994097919124
M12=499.4035693549777
scale = 5.0
height = 292.85
width = 450.75
origin= [-105.599997, -63.249998]

A = np.array([[M00, M01, M02],
              [M10, M11, -M12],
              [0,0,1]])

B = np.array([[1.0, 0.0, width/2.0],
              [0.0, 1.0, height/2.0],
              [0,0,1]])

C = np.array([[1.0, 0.0, -origin[0]],
              [0.0, 1.0, height + origin[1]],
              [0,0,1]])

S = np.array([[scale/100.0, 0.0, 0],
              [0.0, scale/100.0, 0],
              [0,0,1]])

change_base = np.array([[0.0, 0.0, 1.0],
                        [-1.0, 0.0, 0.0],
                        [0,-1.0,0]])

print(np.matmul(np.matmul(A,C),B))