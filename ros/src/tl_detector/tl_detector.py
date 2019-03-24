#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import yaml
# from scipy.spatial import KDTree
import math
from points_organizer import PointsOrganizer


STATE_COUNT_THRESHOLD = 3
MAX_DETECTION_DIST = 100.0
MIN_DETECTION_DIST = 10.0


class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        # self.waypoints_2d = None
        # self.waypoint_tree = None
        self.waypoints_organizer = None
        self.camera_image = None
        self.lights = []
        self.stop_line_organizer = None

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.bridge = CvBridge()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.class_count = 0
        self.process_count = 0

        model_file = rospy.get_param('model_file')
        self.light_classifier = TLClassifier(model_file)
        self.listener = tf.TransformListener()

        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_organizer = PointsOrganizer([[stop_line[0], stop_line[1]] for stop_line in self.stop_line_positions])


        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size=1, buff_size=100 * 1024 * 1024, tcp_nodelay=True)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.traffic_light_color = rospy.Publisher('/traffic_color', String, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        self.waypoints_organizer = PointsOrganizer([[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints])

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
                   of the waypoint closest to the red light's stop line to /traffic_waypoint
               Args:
                   msg (Image): image from car-mounted camera
               """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        temp = self.get_light_state(None)
        rospy.loginfo("COLOR STATE: %s", self.get_color(temp))

        self.traffic_light_color.publish("-----Traffic Light:" + self.get_color(temp) + "-------")

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

        self.state_count += 1

    def get_color(self, state):
        color = "unknown"
        if state == 0:
            color = "red"
        if state == 1:
            color = "yellow"
        if state == 2:
            color = "green"

        return color

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        closest_idx = self.waypoints_organizer.get_closest_point_idx(x, y, look_mode='AHEAD')

        # closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if not self.has_image:
            self.prev_light_loc = None
            return TrafficLight.UNKNOWN


        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # closest_light = None
        line_wp_idx = -1
        state = TrafficLight.UNKNOWN

        if self.pose: # and self.waypoints_organizer and self.stop_line_organizer and self.lights:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            closest_waypoint = self.base_waypoints.waypoints[car_wp_idx]

            # Getting the closest stop line ahead of the vehicle
            closest_stop_line_idx = self.stop_line_organizer.get_closest_point_idx(closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y, look_mode='AHEAD')

            if closest_stop_line_idx is not None:
                closest_light = self.lights[closest_stop_line_idx]

                dist_to_light = math.sqrt((self.pose.pose.position.x - closest_light.pose.pose.position.x)**2 +(self.pose.pose.position.y - closest_light.pose.pose.position.y)**2)
                # If the closest traffic light ahead is not within the maximum distance,
                # skips classifying and publishing it
                if MIN_DETECTION_DIST > dist_to_light > MAX_DETECTION_DIST:
                    return line_wp_idx, state

                # Get stop line waypoint index
                line = self.stop_line_positions[closest_stop_line_idx]

                # Getting the waypoint closest to stop line
                line_wp_idx = self.get_closest_waypoint(line[0], line[1])

                state = self.get_light_state(closest_light)

        return line_wp_idx, state


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')