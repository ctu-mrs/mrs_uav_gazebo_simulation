import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import Waypoint
from mavros_msgs.srv import WaypointPush
from sensor_msgs.msg import NavSatFix

class PixhawkTakeoff:
    
    def callback_global_pos(self, msg):
        if not self.ready:
            return
        rospy.loginfo_once('Got global position')
        self.got_position = True
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def switch_to_mode(self, mode):
        rospy.loginfo('Switching to ' + str(mode))
        self.service_set_mode.call(6, str(mode))

    def call_arming(self):
        rospy.loginfo_once('Arming...')
        msg = CommandBool()
        resp = self.service_arming.call(True)
        return resp.success

    def __init__(self):
        self.ready = False
        self.got_position = False
        self.airborne= False

        rospy.init_node('pixhawk_takeoff', anonymous=True)
        self.takeoff_height = rospy.get_param('~takeoff_height')

        self.subscriber_pos = rospy.Subscriber('~global_pos_in', NavSatFix, self.callback_global_pos)
        self.service_arming = rospy.ServiceProxy('~arming_out', CommandBool)
        self.service_set_mode = rospy.ServiceProxy('~mode_out', SetMode)
        self.service_push_wp = rospy.ServiceProxy('~waypoint_push', WaypointPush)
        
        self.ready = True

        while not self.got_position and not rospy.is_shutdown():
            rospy.sleep(0.1)
            rospy.loginfo_once('Waiting for vehicle position...')

        # #{ Prepare mission
        wp0 = Waypoint()
        wp0.frame = 3 # GLOBAL_REL_ALT
        wp0.command = 22 # TAKEOFF
        wp0.is_current = True
        wp0.x_lat = self.latitude
        wp0.y_long = self.longitude
        wp0.z_alt = self.takeoff_height
        wp0.autocontinue = True

        wp1 = Waypoint()
        wp1.frame = 3 # GLOBAL_REL_ALT
        wp1.command = 17 # NAV_LOITER_UNLIM
        wp1.is_current = False
        wp1.x_lat = self.latitude
        wp1.y_long = self.longitude
        wp1.z_alt = self.takeoff_height
        wp1.autocontinue = True
        # #}
        
        resp = self.service_push_wp.call(0, [wp0, wp1])
        print(resp.success)
        print(resp.wp_transfered)
        self.switch_to_mode('AUTO.MISSION')

        while not self.call_arming() and not rospy.is_shutdown():
            rospy.logerr_once('Arming failed. Retrying...')
            rospy.sleep(0.01)
        rospy.loginfo('Vehicle armed')


if __name__ == '__main__':
    try:
        takeoff_node = PixhawkTakeoff()
    except rospy.ROSInterruptException:
        pass
