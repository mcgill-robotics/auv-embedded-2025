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
    for pwm in range(1508, 1901, 8):  # Changed step to 8
        # Initial arming
        pwm_array[thruster_num] = 1500
        pub.publish(ThrusterMicroseconds(pwm_array))
        time.sleep(15)

        pwm_array[thruster_num] = pwm
        pub.publish(ThrusterMicroseconds(pwm_array))
        print(f"Sent PWM {pwm} to thruster {thruster_num + 1}")

        # Sleep for 3.5 seconds if not 1500, otherwise 15 seconds
        if pwm != 1500:
            time.sleep(3.5)
        else:
            time.sleep(15)

        # Set back to neutral
        pwm_array[thruster_num] = 1500
        pub.publish(ThrusterMicroseconds(pwm_array))
        
        # Sleep for 3.5 seconds when going back to neutral (1500)
        time.sleep(15)

    # Descending sequence
    for pwm in range(1492, 1099, -8):  # Changed step to 8
        pwm_array[thruster_num] = pwm
        pub.publish(ThrusterMicroseconds(pwm_array))
        print(f"Sent PWM {pwm} to thruster {thruster_num + 1}")

        # Sleep for 3.5 seconds if not 1500, otherwise 15 seconds
        if pwm != 1500:
            time.sleep(3.5)
        else:
            time.sleep(15)

        # Set back to neutral
        pwm_array[thruster_num] = 1500
        pub.publish(ThrusterMicroseconds(pwm_array))
        
        # Sleep for 3.5 seconds when going back to neutral (1500)
        time.sleep(15)

    print("Thruster test complete.")
