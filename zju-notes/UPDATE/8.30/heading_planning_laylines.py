
import rospy
from collections import deque
import LatLon as ll
import math
from sailing_robot.msg import position
from shapely.geometry import Point, Polygon
import time
from .navigation import angleSum
from .taskbase import TaskBase
from .heading_planning import TackVoting

# For calculations, lay lines don't extend to infinity.
# This is in m; 10km should be plenty for our purposes.
LAYLINE_EXTENT = 10000
beating_angle_tack = 55

class HeadingPlan(TaskBase):
    def __init__(self, nav,
            waypoint=ll.LatLon(50.742810, 1.014469), # somewhere in the solent
            target_radius=5, tack_voting_radius=15, waypoint_id=None,
            ):
        """Sail towards a waypoint.

        *nav* is a sailing_robot.navigation.Navigation instance.
        
        *waypoint* is a LatLon object telling us where to go.
        
        *target_radius* is how close we need to get to the waypoint, in metres.
        
        *tack_voting_radius* is the distance within which we use tack voting, to
        avoid too frequent tacks close to the waypoint.
        """
        self.target_radius = target_radius
        self.nav = nav
        self.waypoint = waypoint
        x, y = self.nav.latlon_to_utm(waypoint.lat.decimal_degree, waypoint.lon.decimal_degree)
        self.waypoint_xy = Point(x, y)

        self.target_area = self.waypoint_xy.buffer(target_radius)

        self.sailing_state = 'normal'  # sailing state can be 'normal','switch_to_port_tack' or  'switch_to_stbd_tack'
        self.waypoint_id = waypoint_id
        # rospy.logwarn('begin heading_planning')
        self.tackpoint_xy = Point(0,0)
        self.within_tack = 0
        self.tackpoint = position()
        self.pub_tackpoint = rospy.Publisher("tack_point", position, queue_size=10)

    def start(self):
        pass

    def check_end_condition(self):
        """Are we there yet?"""
        return self.nav.position_xy.within(self.target_area)

    debug_topics = [
        ('dbg_heading_to_waypoint', 'Float32'),
        ('dbg_distance_to_waypoint', 'Float32'),
        ('dbg_goal_wind_angle', 'Float32'),
        ('dbg_latest_waypoint_id', 'String'),
    ]

    def calculate_state_and_goal(self):
        """
        Work out what we want the boat to do
        """
        dwp, hwp = self.nav.distance_and_heading(self.waypoint_xy)
        # rospy.logwarn("hwp = " + str(hwp))
        # rospy.logwarn("sin(90) = " + str(math.sin(math.pi / 2)))
        angle_to_wind = self.nav.angle_to_wind()
        # absolute_wind_direction = self.nav.absolute_wind_direction()
        rospy.logwarn("self.within_tack = " + str(self.within_tack))

        self.debug_pub('dbg_distance_to_waypoint', dwp)
        self.debug_pub('dbg_heading_to_waypoint', hwp)
        self.debug_pub('dbg_latest_waypoint_id', self.waypoint_id)
        
        wp_wind_angle = self.nav.heading_to_wind_angle(hwp)
        
        boat_wind_angle = self.nav.angle_to_wind()

        if dwp <= 5:
        #very near the goal, no tacking
            self.sailing_state = 'normal'
            return self.sailing_state, hwp

        if self.within_tack == 1:
            # self.calculate_tack_goal(hwp,dwp)
            tackpoint_target_area = self.tackpoint_xy.buffer(6)
            check_reach_tackpoint = self.nav.position_xy.within(tackpoint_target_area)
            if check_reach_tackpoint:
                rospy.logwarn("reach tack point")
                #reach a tack point
                self.within_tack = 0
                self.sailing_state = 'normal'
                return self.sailing_state, hwp
                #time.sleep(1.5)
            else:
                rospy.logwarn("not reach tack point yet!!!!!!!!!!")
                dwp_tack, hwp_tack = self.nav.distance_and_heading(self.tackpoint_xy)
                rospy.logwarn("dwp_tack = " + str(dwp_tack))
                #give a tack point and calculate its heading        
                self.sailing_state = 'tack'
                return self.sailing_state, hwp_tack
 
        if (wp_wind_angle % 360) > beating_angle_tack   and (wp_wind_angle % 360) < (360-beating_angle_tack):
            self.sailing_state = 'normal'
            rospy.logwarn("in state and goal wp_wind_angle = " + str(wp_wind_angle))
            return self.sailing_state, hwp
        else:
                # Tack should begin or continue
                rospy.logwarn("begin to tack!!!")
                self.within_tack = 1
                self.calculate_tack_goal(hwp,dwp)
                # self.target_area = self.tackpoint_xy.buffer(self.target_radius)
                dwp_tack, hwp_tack = self.nav.distance_and_heading(self.tackpoint_xy)
                self.sailing_state = 'tack'
                return self.sailing_state, hwp_tack



    def calculate_tack_goal(self,hwp,dwp):
        '''calculate which point we will turn while we are tacking
        if we are within 5m around the goal we do not add one more tack point

        '''
        # rospy.logwarn("in calculate_tack_goal")
        wp_wind_angle = self.nav.heading_to_wind_angle(hwp)
        
        # if (wp_wind_angle % 360) > beating_angle_tack + 10  and (wp_wind_angle % 360) < (360-beating_angle_tack-10):
        #     self.tackpoint_xy = self.waypoint_xy

        
        a = beating_angle_tack + 10
        b = wp_wind_angle
        l1 = (dwp / math.sin((180-2*a)*math.pi / 180)) * math.sin((a-b)*math.pi / 180)
        l2 = (dwp / math.sin((180-2*a)*math.pi / 180)) * math.sin((a+b)*math.pi / 180)
        # rospy.logwarn("(dwp / math.sin(180-2*a)) = " + str((dwp / math.sin(180-2*a))))
        # rospy.logwarn("math.sin(a+b) = " + str(math.sin(a+b)))
        # rospy.logwarn("math.sin(a-b) = " + str(math.sin(a-b)))

        # theta = a + hwp
        # M1_dx1 = l1 * math.sin(hwp) - l1 * math.tan(theta) * math.cos(hwp) 
        # M1_dy1 = l1 * math.cos(hwp) + l1 * math.tan(theta) * math.sin(hwp)
        rospy.logwarn("l1 = " + str(l1))
        rospy.logwarn("l2 = " + str(l2))
        rospy.logwarn("dwp = " + str(dwp))

        # rospy.logwarn("first case : M1_dx = " + str(M1_dx1))
        # rospy.logwarn("first case : M1_dy = " + str(M1_dy1))

        M1_dx = abs(l1) * math.cos((90-hwp-a+b)*math.pi / 180)
        M1_dy = abs(l1) * math.sin((90-hwp-a+b)*math.pi / 180)
        # M1_dx = 10
        # M1_dy = -10

        # rospy.logwarn("second case : M1_dx2 = " + str(M1_dx))
        # rospy.logwarn("second case : M1_dy2 = " + str(M1_dy))

        M2_dx = abs(l2) * math.cos((90-hwp+a+b) * math.pi / 180)
        M2_dy = abs(l2) * math.sin((90-hwp+a+b) * math.pi / 180)

        M1 = Point(self.nav.position_xy.x + M1_dx, self.nav.position_xy.y + M1_dy)
        # M2 = Point(self.nav.position_xy.x + M2_dx, self.nav.position_xy.x + M2_dy)

        self.tackpoint_xy = M1
        self.tackpoint.x = self.nav.position_xy.x + M1_dx
        self.tackpoint.y = self.nav.position_xy.y + M1_dy

    # rospy.logwarn(self.nav.position_xy)
    # rospy.logwarn(M1)
        self.pub_tackpoint.publish(self.tackpoint)
    # rospy.logwarn(self.nav.position_xy)
