import numpy as np
import os
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import math
from math import sqrt

enable_3D = True 
enable_2D = True
enable_depth = True

# IMU
# 1. mathematiacally check translations (y_coordinates, z_coordinated interchanged)
# y -y : S R T

# CONST_EULER_ANGLE = [0, 7*np.pi/12, -np.pi/2]

def eulerAnglesToRotationMatrix(theta) :
  R_x = np.array([[1,         0,                  0                   ],
                  [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                  [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                  ])

  R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                  [0,                     1,      0                   ],
                  [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                  ])

  R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                  [math.sin(theta[2]),    math.cos(theta[2]),     0],
                  [0,                     0,                      1]
                  ])

  R = np.dot(R_z, np.dot( R_y, R_x )) 
  ###CHECK


  return R
###########################Ground truths###########################################################


# ground_truth = np.array([[], [], []])

# file1 = open('bag2_outputx.txt', 'r')
# file2 = open('bag2_outputy.txt', 'r')
# file3 = open('bag2_outputz.txt', 'r')

# for line in file1.readlines():
#   ground_truth[0].append(float(line))

# for line2 in file2.readlines():
#   ground_truth[1].append(float(line2))

# for line3 in file3.readlines():
#   ground_truth[2].append(float(line3))  
# X Y Z ARE GT
###########################ORB###########################################################

################################ROTATTION Matrix################################
R = eulerAnglesToRotationMatrix([0, 0, 0]) #rotation matrix

# R = eulerAnglesToRotationMatrix([0, 0, -np.pi/12])
# Q = [1/sqrt(3), 0, 1/sqrt(3), -1/sqrt(3)]
# Q = [0, 0, 0, 1]
# R = quaternion_rotation_matrix(Q)
print(R)

# t = np.array([[0.083, -0.03, -0.045]]) # translations array
t = np.array([[0, 0, 0]])
P = np.matrix(np.vstack((np.hstack((R, t.transpose())),[0, 0, 0, 1]))) # combined transformation matrix
print(P)

inv_P = np.linalg.inv(P) # inverse transformation

orb = np.array([])

output_file = open("out.txt", 'w')
with open('flight1_rgbd_cam_traj.txt' , 'r') as f:
    for line in f.readlines():
      fragment = line.split()
      data = np.array([[float(fragment[1]), float(fragment[2]), float(fragment[3]), 1.0]])

      orb = np.dot(inv_P, data.transpose())
      output_file.write(str(fragment[0]+ ' '))
      output_file.write(str(orb[0])[2:-2] + ' ' + str(orb[1])[2:-2] + ' ' + str(orb[2])[2:-2] + '\n')

output_file.close()


# array2D = [ ]

# # x_coordinates = [0]
# # y_coordinates = [0]
# # z_coordinates = [0]

# orb_coord = np.array([[],[],[]])

# for i in range (len(array2D)):

#     orb_coord[0].append(array2D[i][1])
#     orb_coord[1].append(array2D[i][2])
#     orb_coord[2].append(array2D[i][3])


# orb_coord[0] = [float(x) for x in orb_coord[0]]
# orb_coord[1] = [float(x) for x in orb_coord[1]]
# orb_coord[2] = [float(x) for x in orb_coord[2]]

# # current_pos = [0,0,0]

# # for i in range (len(orb_coord[0])):
# #   current_pos[0] = orb_coord[0][i]
# #   current_pos[1] = orb_coord[1][i]
# #   current_pos[2] = orb_coord[2][i]

# #   new_pos = np.dot(R, current_pos)

# #   orb_coord[0][i] = new_pos[0]
# #   orb_coord[1][i] = new_pos[1]
# #   orb_coord[2][i] = new_pos[2]

# ###########################TRANSLATION+SCALE###########################################################


#   #  x:  8.3 cm
#   #  y: -3.0 cm
#   #  z: -4.5 cm

# # for i in range (len(y_coordinates)):
# #    y_coordinates[i] = -1*y_coordinates[i]

# t = np.array([.083,-.03,-.045]) # translation vector

# #taking the original co-ordinates of drone into account
# # for i in range(len(t)):
# #   t[i] += ground_truth[i][0]



# sx = 1
# sz = 1


# for i in range (len(orb-coord[0])):
#   P = np.stack((orb_coord, ))

#####################################PLOT##################################################################



# print ("inital GROUND TRUTH" , x[0],y[0],z[0] )
# print ("inital ORB " , x_coordinates[0],y_coordinates[0],z_coordinates[0] )
# if enable_3D:
#     plt.style.use('seaborn-poster')
#     fig = plt.figure(figsize = (8,8))
#     ax = plt.axes(projection='3d')
#     ax.grid()


#     ax.plot3D(x_coordinates, z_coordinates, y_coordinates)
#     ax.plot3D(x,y,z)
#     ax.set_title('3D- Trajectory')

#     ax.set_xlabel('x (meters)', labelpad=20)
#     ax.set_ylabel('y', labelpad=20)
#     ax.set_zlabel('z', labelpad=20)

#     plt.show()

# if enable_2D:

#     plt.plot(x_coordinates,z_coordinates, 'r')
#     plt.plot(x,y,'g')
#     ax = plt.axes()
#     ax.grid()
#     plt.axis('scaled')

#     ax.set_xlabel('x (meters)', labelpad=20)
#     ax.set_ylabel('y', labelpad=20)
#     plt.show()

# # y_coordinates2 = []

# # half = int (len(y_coordinates)/2)
# # for i in range (len(y_coordinates)):
# #    if (i<=half):
# #       y_coordinates2.append(-1*y_coordinates[i])
# #    else:
# #       y_coordinates2.append(y_coordinates[i])



# if enable_depth:
#     t1 = np.linspace(0, 1, len(y_coordinates))
#     t2 = np.linspace(0, 1, len(z))

#     fig, ax1 = plt.subplots()
#     color = 'tab:red'
#     ax1.set_xlabel('counts')
#     ax1.set_ylabel('depth_orb', color=color)
#     ax1.plot(t1, y_coordinates, color=color)
#     # ax1.tick_params(axis='y', labelcolor=color)
#     # ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

#     color = 'tab:blue'
#     ax1.set_ylabel('depth_gt', color=color)  # we already handled the x-label with ax1
#     ax1.plot(t2, z, color=color)
#     ax1.tick_params(axis='y', labelcolor=color)

#     fig.tight_layout()  # otherwise the right y-label is slightly clipped
#     plt.show()



