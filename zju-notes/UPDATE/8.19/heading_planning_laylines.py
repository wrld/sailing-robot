import rospy
from collections import deque
import LatLon as ll
import math
from shapely.geometry import Point, Polygon

from .navigation import angleSum
from .taskbase import TaskBase
from .heading_planning import TackVoting

# For calculations, lay lines don't extend to infinity.
# This is in m; 10km should be plenty for our purposes.
LAYLINE_EXTENT = 10000

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
        self.nav = nav
        self.waypoint = waypoint
        x, y = self.nav.latlon_to_utm(waypoint.lat.decimal_degree, waypoint.lon.decimal_degree)
        self.waypoint_xy = Point(x, y)
        self.target_area = self.waypoint_xy.buffer(target_radius)
        self.sailing_state = 'normal'  # sailing state can be 'normal','switch_to_port_tack' or  'switch_to_stbd_tack'
        self.tack_voting = TackVoting(50, 35)
        self.tack_voting_radius = tack_voting_radius
        self.waypoint_id = waypoint_id
        # rospy.logwarn('begin heading_planning')
        self.x = []
        self.y = []

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
        angle_to_wind = self.nav.angle_to_wind()
        absolute_wind_direction = self.nav.absolute_wind_direction()

        # rospy.logwarn('distance =' + str(dwp))
        # rospy.logwarn('heading = ' + str(hwp))
        # rospy.logwarn('waypoint_id = ' + str(self.waypoint_id))
        # rospy.logwarn('angle_to_wind = ' + str(angle_to_wind))
        # rospy.logwarn('absolute_wind_direction = ' + str(absolute_wind_direction))

        self.debug_pub('dbg_distance_to_waypoint', dwp)
        self.debug_pub('dbg_heading_to_waypoint', hwp)
        self.debug_pub('dbg_latest_waypoint_id', self.waypoint_id)
        
        wp_wind_angle = self.nav.heading_to_wind_angle(hwp)
        rospy.logwarn("wp_wind_angle = " + str(wp_wind_angle))
        boat_wind_angle = self.nav.angle_to_wind()
        if self.sailing_state != 'normal':  
            # A tack/jibe is in progress
            if self.sailing_state == 'switch_to_port_tack':
                goal_angle = self.nav.beating_angle
                continue_tack = boat_wind_angle < goal_angle or boat_wind_angle > 120
            else:  # 'switch_to_stbd_tack'
                goal_angle = -self.nav.beating_angle
                continue_tack = boat_wind_angle > goal_angle or boat_wind_angle < -120

            # rospy.logwarn('continue_tack = ' + str(continue_tack))
            if (wp_wind_angle % 360) > 60 and (wp_wind_angle % 360) < 300:
                goal_wind_angle = wp_wind_angle
                rospy.logwarn("in shun feng!!!!!!!!!!!!!!!!!!!")
                self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
                state = 'normal'
                return state, self.nav.wind_angle_to_heading(goal_wind_angle)
                
            if continue_tack:
                self.debug_pub('dbg_goal_wind_angle', goal_angle)

                # rospy.logwarn('continue_tack')
                # rospy.logwarn('boat_wind_angle = ' + str(boat_wind_angle))
                # # rospy.logwarn('wp_wind_angle = ' + str(wp_wind_angle))
                # rospy.logwarn('goal_angle = ' + str(goal_angle))
                # rospy.logwarn('heading_angle = ' + str(self.nav.wind_angle_to_heading(goal_angle)))

                return self.sailing_state, self.nav.wind_angle_to_heading(goal_angle)
            else:
                # Tack completed
                self.log('info', 'Finished tack (%s)', self.sailing_state)
                self.tack_voting.reset(boat_wind_angle > 0)
                self.sailing_state = 'normal'

        on_port_tack = boat_wind_angle > 0


        
        # Detect if the waypoint is downwind, if so head directly to it
        if (wp_wind_angle % 360) > 60 and (wp_wind_angle % 360) < 300:
            goal_wind_angle = wp_wind_angle
            self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
            state = 'normal'
            return state, self.nav.wind_angle_to_heading(goal_wind_angle)

        tack_now = False

        # rospy.logwarn('wp_wind_angle = ' + str(wp_wind_angle))
        # rospy.logwarn('boat_wind_angle = ' + str(boat_wind_angle))
        # rospy.logwarn('on_port_tack = ' + str(on_port_tack))
        flag1 = wp_wind_angle * boat_wind_angle > 0
        flag2 = self.nav.position_xy.within(self.lay_triangle()) 
        poly = self.lay_triangle()

        # rospy.logwarn('distance = '+ str(dwp))
        # rospy.logwarn(' wp_wind_angle * boat_wind_angle > 0 = ' + str(flag1))
        # rospy.logwarn('self.nav.position_xy.within(self.lay_triangle()) = ' + str(flag2))
        # rospy.logwarn('self.lay_triangle() = ' + str(poly))
        if wp_wind_angle * boat_wind_angle > 0:
            # These two have the same sign, so we're on the better tack already
            self.tack_voting.vote(on_port_tack)
        elif self.nav.position_xy.within(self.lay_triangle()):
            # We're between the laylines; stick to our current tack for now
            self.tack_voting.vote(on_port_tack)
        else:
            tack_now = True
            self.tack_voting.vote(not on_port_tack)

        votes_sum = self.tack_voting.get_votesum()

        # rospy.logwarn('votes_sum = ' + str(votes_sum))
        if dwp < self.tack_voting_radius:
            # Close to the waypoint, use tack voting so we're not constantly
            # tacking.
            # rospy.logwarn('within tack voting')
            tack_now = self.tack_voting.tack_now(on_port_tack)

        # rospy.logwarn('tack_now = '+ str(tack_now))

        if tack_now:
            # rospy.logwarn('begin tack')
            # Ready about!
            if on_port_tack:
                state = 'switch_to_stbd_tack'
                goal_wind_angle = -self.nav.beating_angle
            else:
                state = 'switch_to_port_tack'
                goal_wind_angle = self.nav.beating_angle
            self.sailing_state = state
            self.log('info', 'Starting tack/jibe (%s)', state)
        else:
            # rospy.logwarn('Stay on our current tack')
            # Stay on our current tack
            if on_port_tack:
                goal_wind_angle = max(wp_wind_angle, self.nav.beating_angle)
            else:
                goal_wind_angle = min(wp_wind_angle, -self.nav.beating_angle)
            state = 'normal'
        # rospy.logwarn('state = ' + str(state))

        self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
["A"]
        # rospy.logwarn('tack')
        # rospy.logwarn('boat_wind_angle = ' + str(boat_wind_angle))
        # rospy.logwarn('wp_wind_angle = ' + str(wp_wind_angle))
        # rospy.logwarn('goal_angle = ' + str(goal_wind_angle))
        # rospy.logwarn('heading_angle = ' + str(self.nav.wind_angle_to_heading(goal_wind_angle)))

        return state, self.nav.wind_angle_to_heading(goal_wind_angle)

    def lay_triangle(self):
        """Calculate the lay lines for the current waypoint.
        
        This returns a shapely Polygon with the two lines extended to
        LAYLINE_EXTENT (10km).
        """
        
        # rospy.logwarn('absolute_wind_direction = ' + str(self.nav.absolute_wind_direction()))
        downwind = angleSum(self.nav.absolute_wind_direction(), 180)
        # rospy.logwarn('downwind = ' + str(downwind))
        ting_angle = 0
        x0, y0 = self.waypoint_xy.x, self.waypoint_xy.y
        l1 = math.radians(angleSum(downwind, -ting_angle))
        # rospy.logwarn('l1_degree = ' + str(angleSum(downwind, -self.nav.beating_angle)))
        # rospy.logwarn('beating_angle = ' + str(self.nav.beating_angle))
        # rospy.logwarn('l1_radians = ' + str(l1))
        x1 = x0 + (LAYLINE_EXTENT * math.sin(l1))
        y1 = y0 + (LAYLINE_EXTENT * math.cos(l1))
        l2 = math.radians(angleSum(downwind, ting_angle))
        # rospy.logwarn('l2_degree = ' + str(angleSum(downwind, self.nav.beating_angle)))
        # rospy.logwarn('l2_radians = ' + str(l2))
        x2 = x0 + (LAYLINE_EXTENT * math.sin(l2))
        y2 = y0 + (LAYLINE_EXTENT * math.cos(l2))
        self.x = [x0, x1, x2]
        self.y = [y0, y1, y2]
        return Polygon([(x0, y0), (x1, y1), (x2, y2)])
