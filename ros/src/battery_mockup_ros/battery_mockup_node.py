import rospy
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Bool

import numpy as np

class Battery:
    """
    """
    def __init__(self):
        # register node in ROS network
        rospy.init_node('~single_robot_battery_mockup', anonymous=False)

        # setup publisher with the battery state
        self.battery_publisher = rospy.Publisher('~battery_state', sensor_msgs.BatteryState, queue_size=1)
        # subscibe to the is_charging topic
        rospy.Subscriber('~is_charging', Bool, self.chargingCallback)

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rospy.Time.now()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 1
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        # is the robot currently charging
        self.is_charging = True

        if rospy.has_param('~battery_autonomy'):
            # retrieves the battery autonomy
            self.battery_autonomy = rospy.get_param('~battery_autonomy')
        else:
            self.battery_autonomy = 30

        print('Battery autonomy: ', self.battery_autonomy)

        # battery half life and total life
        self.bat_half_life = (10.0-np.log(1))/(np.log(2)/(self.battery_autonomy/2)*10)
        self.bat_max_life = 2*self.bat_half_life

        # Set the initial battery level
        # Discharged
        # self.seconds_elapsed = 0
        # Choice between 0, half or fully charged
        # self.seconds_elapsed = np.random.choice([0, self.bat_half_life, self.bat_max_life])
        # Uniform distribution
        self.seconds_elapsed = np.random.uniform(0, self.bat_max_life)
        print('Seconds chosen : ', self.seconds_elapsed)

        self.white_noise = True

    # Battery charge model
    def battery_charge(self, time, battery_autonomy):
        if self.white_noise:
            battery_autonomy = np.random.normal(loc=battery_autonomy, scale=3)
        lb = np.log(2)/(battery_autonomy/2)*10
        return 1/(1+np.exp(-lb*time+10))

    # Battery discharge model
    def battery_discharge(self, time, battery_autonomy):
        if self.white_noise:
            battery_autonomy = np.random.normal(loc=battery_autonomy, scale=3)
        lb = np.log(2)/(battery_autonomy/2)*10
        return 1/-(1+np.exp(-lb*time+10))+1

    # Called when robot changes state, from charging to discharging or vice-versa
    def chargingCallback(self, charging_state):
        if self.is_charging != bool(charging_state.data):
            self.is_charging = bool(charging_state.data)
            # mirror the time elapsed to obtain the exact correspondence between the discharge and charge model timestamp
            self.seconds_elapsed = self.bat_half_life+(self.bat_half_life-self.seconds_elapsed)

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        rate = rospy.Rate(1)  # hz
        while not rospy.is_shutdown():
            if self.is_charging:
                self.battery.percentage = min(1, self.battery_charge(self.seconds_elapsed, self.battery_autonomy))
                self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                self.battery.percentage = max(0, self.battery_discharge(self.seconds_elapsed, self.battery_autonomy))
                self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

            self.battery.header.stamp = rospy.get_rostime()
            self.battery_publisher.publish(self.battery)
            rate.sleep()

            self.seconds_elapsed = min(self.bat_max_life, self.seconds_elapsed+1)

def main():
    battery = Battery()
    battery.spin()