import rospy
import sensor_msgs.msg as sensor_msgs
from std_msgs.msg import Bool

class Battery:
    """
    """
    def __init__(self):
        # ros communications
        self.battery_publisher = rospy.Publisher('/battery/sensor_state', sensor_msgs.BatteryState, queue_size=1, latch=True)
        rospy.Subscriber('/battery/is_charging', Bool, self.chargingCallback)

        # initialisations
        self.battery = sensor_msgs.BatteryState()
        self.battery.header.stamp = rospy.Time.now()
        self.battery.voltage = float('nan')
        self.battery.current = float('nan')
        self.battery.charge = float('nan')
        self.battery.capacity = float('nan')
        self.battery.design_capacity = float('nan')
        self.battery.percentage = 100
        self.battery.power_supply_health = sensor_msgs.BatteryState.POWER_SUPPLY_HEALTH_GOOD
        self.battery.power_supply_technology = sensor_msgs.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        self.battery.power_supply_status = sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_FULL
        self.battery.present = True
        self.battery.location = ""
        self.battery.serial_number = ""

        self.is_charging = False
        self.start_time = rospy.get_rostime()

        if rospy.has_param('/battery/battery_autonomy'):
            # retrieves the threshold from the parameter server in the case where the parameter exists
            self.battery_autonomy = rospy.get_param('/battery/battery_autonomy')
        else:
            self.battery_autonomy = 100

    def battery_charge(time, battery_autonomy):
        lb = np.log(2)/(battery_autonomy/2)*10
        return 1/(1+np.exp(-lb*time+10))

    def battery_discharge(time, battery_autonomy):
        lb = np.log(2)/(battery_autonomy/2)*10
        return 1/-(1+np.exp(-lb*time+10))+1

    def chargingCallback(self, is_charging):
        if self.is_charging != is_charging:
            self.is_charging = is_charging
            self.start_time = rospy.get_rostime()

    def spin(self):
        """
        Spin around, updating battery state and publishing the result.
        """
        rate = rospy.Rate(1)  # hz
        while not rospy.is_shutdown():
            elapsed_time = rospy.get_rostime - self.start_time
            print(elapsed_time)
            # if self.is_charging:
            #     self.battery.percentage = min(100, battery_charge(time_charge_start, self.battery_autonomy))

            #     print('Charging')
            #     print('bat level: ', self.battery.percentage)
            # else:
            #     self.battery.percentage = max(0, battery_discharge(time_since_last_charge, self.battery_autonomy))

            #     print('Discharging')
            #     print('bat level: ', self.battery.percentage)

            # self.battery.header.stamp = rospy.get_rostime()  # get_rostime() returns the time in rospy.Time structure
            # self.battery_publisher.publish(self.battery)
            rate.sleep()