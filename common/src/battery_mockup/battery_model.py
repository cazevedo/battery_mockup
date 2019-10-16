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
    bat_aut = 30
    lb = np.log(2)/(bat_aut/2)*10
    half_life = (10-np.log(1))/lb

    time = np.arange(0,40,1)
    charge_level = battery_charge(time, battery_autonomy=bat_aut)
    dicharge_level = battery_discharge(time, battery_autonomy=bat_aut)

    t = 25
    print(t, battery_charge(t, battery_autonomy=bat_aut))
    print(half_life+(half_life-t), battery_discharge(half_life+(half_life-t), battery_autonomy=bat_aut))

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