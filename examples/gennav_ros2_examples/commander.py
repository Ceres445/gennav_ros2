import gennav
import rclpy
from .conversions import Odom_to_RobotState, traj_to_msg
from .utils import get_funcs
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory


class Commander(rclpy.node.Node):
    """Coordinates planning and control

    Args:
        planner (gennav.planners.Planner): object of the Planner class for path planning algorithms
        env (gennav.envs.Environment): object of the Environment class for an environment
        msg_dtype (ROS Message): Type of message used for envrionment data
        replan_interval (int (seconds)): Desired period between callbacks. Defaults to 1s
    """

    def __init__(
        self,
        planner,
        env,
        msg_dtype,
        replan_interval=1,
    ):
        # Initialise commander node
        super().__init__("commander")

        # Store attributes
        self.planner = planner
        self.env = env
        self.replan_interval = replan_interval
        self.msg_dtype = msg_dtype

        # Get conversion and transformation functions for environment
        # data according to types of environment and message data
        self.msg_to_env_data, self.transform_env_data = get_funcs(
            self.env, self.msg_dtype
        )

        # Initialise variables
        self.curr_state = gennav.utils.RobotState()

        # Subscribe to envrionment data topic
        self._env_sub = self.create_subscription(
            self.msg_dtype, "/gennav/env", self._env_cb, 10
        )

        # Subscribe to odometry
        self._odom_sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        # Publisher for planned trajectory
        self._traj_pub = self.create_publisher(
            MultiDOFJointTrajectory, "/gennav/traj", 10
        )

    def goto(self, goal, start=None):
        """Method to find Trajectory of the robot using the planner.plan method.
        Args:
            goal (gennav.utils.RobotState): A RobotState() object which is passed to the planner.plan method.
            start (gennav.utils.RobotState, optional): A RobotState() object which can be passed to the planner.plan method. Defaults to None.
        """
        # Set current state as start if not specified
        start = self.curr_state if start is not None else start

        # Compute first planned trajectory
        print("Planning")
        self.traj, _ = self.planner.plan(start, goal, self.env)
        print("Done Planning")

        # Publish first planned trajectory
        self._publish_traj(self.traj)

        def replan():
            """Method to plan the path again using the planner.replan method."""
            if not self.env.get_traj_status(self.traj):
                self.traj, _ = self.planner.replan(self.curr_state, goal, self.env)
                self._publish_traj(self.traj)

        # Keep replanning at specified intervals
        self.timer = self.create_timer(
            self.replan_interval, replan
        )

    def _publish_traj(self, traj):
        """Method to publish the velocities on /cmd_vel topic.
        Args:
            traj (gennav.utils.Trajectory): Trajectory planned by the planner
        """
        traj_msg = traj_to_msg(traj)
        self._traj_pub.publish(traj_msg)

    def _env_cb(self, msg):
        """Callback function for environment
        Args:
            msg (self.msg_dtype): Subscribed to ROS msg data of the robot on /gennav/env topic
        """
        data = self.msg_to_env_data(msg)
        data = self.transform_env_data(data, self.curr_state)
        self.env.update(data)

    def _odom_cb(self, msg):
        """Callback function for odometry
        Args:
            msg (Odometry): Subscribed Odometry of the robot on /odom topic
        """
        self.curr_state = Odom_to_RobotState(msg)
