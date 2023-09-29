from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import os


ground_truth = [[], [], []]

# file1 = open('/home/metrics/icuas/useless/outputx.txt', 'r')
# file2 = open('/home/metrics/icuas/useless/outputy.txt', 'r')
# file3 = open('/home/metrics/icuas/useless/outputz.txt', 'r')

# for line in file1.readlines():
#   ground_truth[0].append(float(line))

# for line2 in file2.readlines():
#   ground_truth[1].append(float(line2))

# for line3 in file3.readlines():
#   ground_truth[2].append(float(line3))
file1 = open('bag1_gt.txt', 'r')
for line in file1.readlines():
  data=line.split()
  ground_truth[0].append(float(data[1]))
  ground_truth[1].append(float(data[2]))
  ground_truth[2].append(float(data[3]))
orb_file = open("/home/metrics/run/pose.txt", 'r')
orb_data = orb_file.readlines()
orb_x = []
orb_y = []
orb_z = []

for data in orb_data:
    info = data.split()
    # print (info)
    orb_x.append(float(info[1]))
    orb_y.append(float(info[2]))
    orb_z.append(float(info[3]))



#Plot 3D

# plt.style.use('seaborn-poster')
# fig = plt.figure(figsize = (8,8))
# ax = plt.axes(projection='3d')
# ax.grid()


# ax.plot3D(orb_x, orb_y, orb_z)
# ax.plot3D(ground_truth[0],ground_truth[1],ground_truth[2])
# ax.set_title('3D- Trajectory')

# ax.set_xlabel('x (meters)', labelpad=20)
# ax.set_ylabel('y', labelpad=20)
# ax.set_zlabel('z', labelpad=20)

# plt.show()  


#2D Plot

# plt.plot(orb_x,orb_y, 'r')
# plt.plot(ground_truth[0],ground_truth[1],'g')
# ax = plt.axes()
# ax.grid()
# plt.axis('scaled')

# ax.set_xlabel('x (meters)', labelpad=20)
# ax.set_ylabel('y', labelpad=20)
# plt.show()

#x-Comparision plot

# plt.plot(orb_x[:100],orb_y[:100],'r')
# plt.plot(ground_truth[0][:100],ground_truth[1][:100],'g')
# plt.xlabel('x axis')
# plt.ylabel('y axis')
# plt.show()


plt.style.use('seaborn-poster')
fig = plt.figure(figsize = (8,8))
ax = plt.axes(projection='3d')
ax.grid()


ax.plot3D(orb_x, orb_y, orb_z)
ax.plot3D(ground_truth[0],ground_truth[1],ground_truth[2])
ax.set_title('3D- Trajectory')

ax.set_xlabel('x (meters)', labelpad=20)
ax.set_ylabel('y', labelpad=20)
ax.set_zlabel('z', labelpad=20)

plt.show()