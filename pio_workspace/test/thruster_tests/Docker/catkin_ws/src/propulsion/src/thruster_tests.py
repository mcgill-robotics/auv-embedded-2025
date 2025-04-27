#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds
import time

if __name__ == "__main__":
    rospy.init_node("thruster_tester")

    pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)

    thruster_num = int(input("Enter the thruster number to test (1-8): ")) - 1

    if thruster_num not in range(8):
        print("Invalid thruster number. Please choose a number between 1 and 8.")
        exit(1)

    print(f"Testing thruster {thruster_num + 1}...")

    # Starting neutral array
    pwm_array = [1500] * 8

    # Ascending sequence
    for pwm in range(1520, 1901, 20):
        pwm_array[thruster_num] = pwm
        pub.publish(ThrusterMicroseconds(pwm_array))
        print(f"Sent PWM {pwm} to thruster {thruster_num + 1}")
        time.sleep(1.5)

        # Set back to neutral
        pwm_array[thruster_num] = 1500
        pub.publish(ThrusterMicroseconds(pwm_array))
        time.sleep(1.5)

    # Descending sequence
    for pwm in range(1480, 1099, -20):
        pwm_array[thruster_num] = pwm
        pub.publish(ThrusterMicroseconds(pwm_array))
        print(f"Sent PWM {pwm} to thruster {thruster_num + 1}")
        time.sleep(1.5)

        # Set back to neutral
        pwm_array[thruster_num] = 1500
        pub.publish(ThrusterMicroseconds(pwm_array))
        time.sleep(1.5)

    print("Thruster test complete.")

