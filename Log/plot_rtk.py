import numpy as np
import matplotlib.pyplot as plt

a_grdt=np.loadtxt('pos_rtk.txt')
a_out=np.loadtxt('mat_out.txt')
#######for normal#######
fig, axs = plt.subplots(3,1)
lab_grdt = ['', 'gt-x', 'gt-y', 'gt-z']
lab_out = ['', 'out-x', 'out-y', 'out-z']
plot_ind = range(7,10)
time=a_out[:,0]
time1 = a_grdt[:,0]
axs[0].set_title('East [m]')
axs[1].set_title('North [m]')
axs[2].set_title('Up [m]')
for i in range(1,4):
    axs[i-1].plot(time1, a_grdt[:,i],'.-', label=lab_grdt[i])
    axs[i-1].plot(time, a_out[:,i+3],'.-', label=lab_out[i])
for j in range(1,4):
    axs[j-1].grid()
    axs[j-1].legend()
plt.grid()
#######for normal#######

fig, axs = plt.subplots(2,1)
axs[0].set_title('clock drift')
lab_1 = ['dt-g', 'dt-r', 'dt-e', 'dt-c']
for i in range(3):
    #if i==1:
    	axs[0].plot(time, a_out[:,i+13],'.-', label=lab_1[i])
axs[0].plot(time, a_out[:,19],'.-', label=lab_1[3])
for i in range(1):
    #axs[i].set_xlim(386,389)
    axs[i].grid()
    axs[i].legend()
plt.grid()

plt.show()
