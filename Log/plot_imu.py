# import matplotlib
# matplotlib.use('Agg')
import numpy as np
import matplotlib.pyplot as plt

#### Draw IMU data
fig, axs = plt.subplots(2)
imu=np.loadtxt('imu_pbp.txt')
time=imu[:,0]
axs[0].set_title('Gyroscope')
axs[1].set_title('Accelerameter')
lab_1 = ['gyr-x', 'gyr-y', 'gyr-z']
lab_2 = ['acc-x', 'acc-y', 'acc-z']
for i in range(3):
    #if i==1:
    	axs[0].plot(time, imu[:,i+1],'.-', label=lab_1[i])
    	axs[1].plot(time, imu[:,i+4],'.-', label=lab_2[i])
for i in range(2):
    #axs[i].set_xlim(386,389)
    axs[i].grid()
    axs[i].legend()
plt.grid()

#fig, axs = plt.subplots(5)
#axs[0].set_title('miss')
#axs[1].set_title('miss')
#axs[2].set_title('miss')
#axs[3].set_title('miss')
#axs[4].set_title('miss')
#len_time1 = np.arange(0,1977)
#len_time2 = np.arange(1977, 3954)
#len_time3 = np.arange(3954,5931)
#len_time4 = np.arange(5931,7908)
#len_time5 = np.arange(7908,9885)
    #if i==1:
#axs[0].plot(len_time1, time[0:1977],'.-', label='check')
#axs[1].plot(len_time2, time[1977:3954],'.-', label='check')
#axs[2].plot(len_time3, time[3954:5931],'.-', label='check')
#axs[3].plot(len_time4, time[5931:7908],'.-', label='check')
#axs[4].plot(len_time5, time[7908:9885],'.-', label='check')
    #axs[i].set_xlim(386,389)
#axs[0].grid()
#axs[0].legend()
#axs[1].grid()
#axs[1].legend()
#axs[2].grid()
#axs[2].legend()
#axs[3].grid()
#axs[3].legend()
#axs[4].grid()
#axs[4].legend()
#plt.grid()

#fig, axs = plt.subplots(5)
#axs[0].set_title('miss')
#axs[1].set_title('miss')
#axs[2].set_title('miss')
#axs[3].set_title('miss')
#axs[4].set_title('miss')
#len_time1 = np.arange(9885,9885+1977)
#len_time2 = np.arange(9885+1977,9885+3954)
#len_time3 = np.arange(9885+3954,9885+5931)
#len_time4 = np.arange(9885+5931,9885+7908)
#len_time5 = np.arange(9885+7908,9885+9885)
    #if i==1:
#axs[0].plot(len_time1, time[9885+0:9885+1977],'.-', label='check')
#axs[1].plot(len_time2, time[9885+1977:9885+3954],'.-', label='check')
#axs[2].plot(len_time3, time[9885+3954:9885+5931],'.-', label='check')
#axs[3].plot(len_time4, time[9885+5931:9885+7908],'.-', label='check')
#axs[4].plot(len_time5, time[9885+7908:9885+9885],'.-', label='check')
    #axs[i].set_xlim(386,389)
#axs[0].grid()
#axs[0].legend()
#axs[1].grid()
#axs[1].legend()
#axs[2].grid()
#axs[2].legend()
#axs[3].grid()
#axs[3].legend()
#axs[4].grid()
#axs[4].legend()
#plt.grid()

# #### Draw time calculation
# plt.figure(3)
# fig = plt.figure()
# font1 = {'family' : 'Times New Roman',
# 'weight' : 'normal',
# 'size'   : 12,
# }
# c="red"
# a_out1=np.loadtxt('Log/mat_out_time_indoor1.txt')
# a_out2=np.loadtxt('Log/mat_out_time_indoor2.txt')
# a_out3=np.loadtxt('Log/mat_out_time_outdoor.txt')
# # n = a_out[:,1].size
# # time_mean = a_out[:,1].mean()
# # time_se   = a_out[:,1].std() / np.sqrt(n)
# # time_err  = a_out[:,1] - time_mean
# # feat_mean = a_out[:,2].mean()
# # feat_err  = a_out[:,2] - feat_mean
# # feat_se   = a_out[:,2].std() / np.sqrt(n)
# ax1 = fig.add_subplot(111)
# ax1.set_ylabel('Effective Feature Numbers',font1)
# ax1.boxplot(a_out1[:,2], showfliers=False, positions=[0.9])
# ax1.boxplot(a_out2[:,2], showfliers=False, positions=[1.9])
# ax1.boxplot(a_out3[:,2], showfliers=False, positions=[2.9])
# ax1.set_ylim([0, 3000])

# ax2 = ax1.twinx()
# ax2.spines['right'].set_color('red')
# ax2.set_ylabel('Compute Time (ms)',font1)
# ax2.yaxis.label.set_color('red')
# ax2.tick_params(axis='y', colors='red')
# ax2.boxplot(a_out1[:,1]*1000, showfliers=False, positions=[1.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out2[:,1]*1000, showfliers=False, positions=[2.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.boxplot(a_out3[:,1]*1000, showfliers=False, positions=[3.1],boxprops=dict(color=c),capprops=dict(color=c),whiskerprops=dict(color=c))
# ax2.set_xlim([0.5, 3.5])
# ax2.set_ylim([0, 100])

# plt.xticks([1,2,3], ('Outdoor Scene', 'Indoor Scene 1', 'Indoor Scene 2'))
# # # print(time_se)
# # # print(a_out3[:,2])
# plt.grid()
# plt.savefig("time.pdf", dpi=1200)
plt.show()
