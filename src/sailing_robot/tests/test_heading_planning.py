import unittest
from nose.tools import assert_equal

from LatLon import LatLon
from sailing_robot.heading_planning import HeadingPlan, TackVoting
from sailing_robot.navigation import Navigation, angleAbsDistance

class DummyNSF(object):
    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

class HeadingPlanTests(unittest.TestCase):
    def setUp(self):
        nav = Navigation(beating_angle=45)
        self.hp = HeadingPlan(nav, waypoint=LatLon(50.7, -0.98),
                            tack_line_offset=0.01)
        self.hp.nav.update_position(DummyNSF(50.7, -1.02))
        # Should head east

    def test_complete_tack_to_port(self):
        self.hp.sailing_state = 'tack_to_port_tack'
        self.hp.nav.wind_direction = 50

        self.hp.nav.heading = 310
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')
        self.assertEqual(goal, 45)

        self.hp.nav.wind_direction = 350
        self.hp.nav.heading = 10
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')
        self.assertEqual(goal, 45)

        self.hp.nav.wind_direction = 313
        self.hp.nav.heading = 47
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertGreater(goal, 45)

    def test_continue_tack_to_stbd(self):
        self.hp.sailing_state = 'tack_to_stbd_tack'
        self.hp.nav.heading = 200
        self.hp.nav.wind_direction = 340  # Wind from the south
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')
        self.assertEqual(goal, 135)

        self.hp.nav.wind_direction = 10
        self.hp.nav.heading = 170
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')
        self.assertEqual(goal, 135)

        self.hp.nav.wind_direction = 48
        self.hp.nav.heading = 132
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertLess(goal, 135)

    def test_plain_sailing(self):
        self.hp.nav.wind_direction = 260
        self.hp.nav.heading = 110
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        # Heading ~= 90, but not exactly, because LatLon calculations are
        # not on a plane.
        self.assertGreater(goal, 89)
        self.assertLess(goal, 91)

    def test_tack_to_port(self):
        self.hp.nav.wind_direction = 90
        self.hp.nav.heading = 280  # We're reaching the wrong way!
        self.hp.tack_voting.votes_sum = 75  # On tack threshold
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')

    def test_tack_to_stbd(self):
        self.hp.nav.wind_direction = 270
        self.hp.nav.heading = 280  # We're reaching the wrong way!
        self.hp.tack_voting.votes_sum = 25  # On tack threshold
        self.hp.tack_voting.votes.extend([1] * 100)
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')

    def test_to_windward_port_tack(self):
        self.hp.nav.wind_direction = 270
        self.hp.nav.heading = 180
        self.hp.tack_voting.votes_sum = 45  # Below tack threshold
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertEqual(goal, 135)

        # Time to switch tack
        self.hp.tack_voting.votes_sum = 15  # Beyond tack threshold
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')

    def test_to_windward_stbd_tack(self):
        self.hp.nav.wind_direction = 90
        self.hp.nav.heading = 0
        self.hp.tack_voting.votes_sum = 45  # Below tack threshold
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertEqual(goal, 45)

        # Time to switch tack
        self.hp.tack_voting.votes_sum = 85  # Above tack threshold
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')

    def test_end_condition(self):
        assert not self.hp.check_end_condition()
        self.hp.nav.update_position(DummyNSF(50.7, -0.9800001))
        assert self.hp.check_end_condition()

    def test_distance_heading_to_waypoint(self):
        d, h = self.hp.distance_heading_to_waypoint()
        self.assertGreater(d, 2500)
        self.assertLess(d, 3000)
        self.assertGreater(h, 85)
        self.assertLess(h, 95)

class TackVotingTests(unittest.TestCase):
    def setUp(self):
        self.tv = TackVoting(100, 75)

    def test_subtract_one(self):
        self.tv.reset(1)
        self.tv.vote(0)
        self.assertEqual(self.tv.votes_sum, 99)

    def test_threshold_to_port(self):
        self.tv.reset(0)
        for a in range(75):
            self.tv.vote(1)
        assert not self.tv.tack_now(0)
        self.tv.vote(1)
        assert self.tv.tack_now(0)

    def test_threshold_to_starboard(self):
        self.tv.reset(1)
        for a in range(75):
            self.tv.vote(0)
        assert not self.tv.tack_now(1)
        self.tv.vote(0)
        assert self.tv.tack_now(1)
