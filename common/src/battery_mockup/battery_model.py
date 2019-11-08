import numpy as np

import seaborn as sns
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
# from matplotlib.ticker import MaxNLocator

def battery_charge(time, battery_autonomy):
    battery_autonomy = np.random.normal(battery_autonomy, battery_autonomy/20)
    lb = np.log(2)/(battery_autonomy/2)*10
    return 1/(1+np.exp(-lb*time+10))

def battery_discharge(time, battery_autonomy):
    battery_autonomy = np.random.normal(battery_autonomy, battery_autonomy/20)
    lb = np.log(2)/(battery_autonomy/2)*10
    return 1/-(1+np.exp(-lb*time+10))+1

if __name__ == '__main__':
    bat_aut = 200.0
    lb = np.log(2)/(bat_aut/2)*10
    print(lb)
    half_life = (10-np.log(1))/lb

    time = np.arange(0,250,1)
    charge_level_1 = battery_charge(time, battery_autonomy=bat_aut)
    charge_level_2 = battery_charge(time, battery_autonomy=bat_aut)
    charge_level_3 = battery_charge(time, battery_autonomy=bat_aut)
    dicharge_level_1 = battery_discharge(time, battery_autonomy=bat_aut)
    dicharge_level_2 = battery_discharge(time, battery_autonomy=bat_aut)
    dicharge_level_3 = battery_discharge(time, battery_autonomy=bat_aut)
    exp_charge = np.exp(-0.015*time)

    t = 200
    print(t, battery_charge(t, battery_autonomy=bat_aut))
    print(half_life+(half_life-t), battery_discharge(half_life+(half_life-t), battery_autonomy=bat_aut))

    sns.set()

    plt.figure()
    # plt.plot(time, level, 'bo-')
    plt.plot(time, charge_level_1)
    plt.plot(time, charge_level_2)
    plt.plot(time, charge_level_3)
    plt.plot(time, dicharge_level_1)
    plt.plot(time, dicharge_level_2)
    plt.plot(time, dicharge_level_3)
    plt.plot(time, exp_charge)

    plt.xlabel("Time (minutes)")
    plt.ylabel("Battery level")
    plt.show()

    # plt.figure()
    # # plt.plot(time, level, 'bo-')
    # plt.plot(time, dicharge_level)
    # plt.xlabel("Time (minutes)")
    # plt.ylabel("Battery level")
    # plt.show()