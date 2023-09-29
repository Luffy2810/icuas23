from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import os


ground_truth = [[], [], []]


file1 = open('/home/metrics/run/icuas/useless/bag3_gt.txt', 'r')
file2 = open('/home/metrics/run/pose.txt', 'r')

gt=file1.readlines()

pose=file2.readlines()

len_pose=len(pose)
len_gt=len(gt)
print (len_pose,len_gt)
init_pose=pose[0]
init_gt=gt[0]
t_1=init_pose[0]
t_0=init_gt[0]
t_diff=float(t_1)-float(t_0)
i=0
j=0
rmse=0
while (i<len_pose and j<len_gt):
  gt_data=gt[j].split()

  pose_data=pose[i].split()

  if (float(pose_data[0])-float(gt_data[0])-t_diff)<0.1:
    print  (gt_data)
    print (pose_data)
    print (i)
    print ('*'*100)
    rmse
    i+=1
    error=((float(gt_data[3])-float(pose_data[3]))**2 + (float(gt_data[1])-float(pose_data[1]))**2 + (float(gt_data[2])-float(pose_data[2]))**2)**0.5
    rmse+=error  
  else:
    j+=1 
    print((float(pose_data[0])-t_diff),(float(gt_data[0])))
rmse=rmse/(i+1)    
print  (rmse)