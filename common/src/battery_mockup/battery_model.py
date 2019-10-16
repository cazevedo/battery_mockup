import numpy as np

import seaborn as sns
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
# from matplotlib.ticker import MaxNLocator

def battery_charge(time, battery_autonomy):
    lb = np.log(2)/(battery_autonomy/2)*10
    return 1/(1+np.exp(-lb*time+10))

def battery_discharge(time, battery_autonomy):
    lb = np.log(2)/(battery_autonomy/2)*10
    return 1/-(1+np.exp(-lb*time+10))+1

if __name__ == '__main__':
    time = np.arange(0,20,0.1)
    charge_level = battery_charge(time, battery_autonomy=7)
    dicharge_level = battery_discharge(time, battery_autonomy=7)

    # print(level)

    sns.set()

    plt.figure()
    # plt.plot(time, level, 'bo-')
    plt.plot(time, charge_level)
    plt.plot(time, dicharge_level)

    plt.xlabel("Time (minutes)")
    plt.ylabel("Battery level")
    plt.show()

    # plt.figure()
    # # plt.plot(time, level, 'bo-')
    # plt.plot(time, dicharge_level)
    # plt.xlabel("Time (minutes)")
    # plt.ylabel("Battery level")
    # plt.show()