import rospy
from geometry_msgs.msg import Twist
from teleop_twist_keyboard import teleop_twist_keyboard
import threading

# Initialize the global velocity message
vel_msg = Twist()

class Commander:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('commander', anonymous=True)

        # Create a publisher for the '/yolobot/cmd_vel' topic
        self.publisher = rospy.Publisher('/yolobot/cmd_vel', Twist, queue_size=10)

        # Set the timer for periodic publishing
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)

    def timer_callback(self, event):
        """Publish the velocity message at regular intervals."""
        self.publisher.publish(vel_msg)

def teleop_control():
    """Function to handle keyboard input and update the velocity message."""
    global vel_msg
    while not rospy.is_shutdown():
        # Get key input and update the velocity message
        key = teleop_twist_keyboard.get_key()
        rospy.loginfo(f"Key pressed: {key}")  # Log key pressed
        if key == 'w':
            vel_msg.linear.x = 1.0  # move forward
        elif key == 's':
            vel_msg.linear.x = -1.0  # move backward
        elif key == 'a':
            vel_msg.angular.z = 1.0  # turn left
        elif key == 'd':
            vel_msg.angular.z = -1.0  # turn right
        elif key == 'q':
            rospy.signal_shutdown("Quit command received")  # shutdown on pressing 'q'

        # Stop the robot if no key is pressed
        elif key == '':
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

def main():
    try:
        # Initialize the Commander
        commander = Commander()

        # Start the teleop_control loop to handle keyboard input
        control_thread = threading.Thread(target=teleop_control)
        control_thread.daemon = True  # Allow thread to exit when main program exits
        control_thread.start()

        # Use a separate thread for spinning
        spin_thread = threading.Thread(target=rospy.spin)
        spin_thread.start()

        # Maintain the loop at a specific rate
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

