import math
from time import time
from gennav.utils.common import Velocity
from rclpy import duration
from std_msgs.msg import Header

import transforms3d
from gennav import utils as utils
from gennav.utils import RobotState, Trajectory
from geometry_msgs.msg import Point, Quaternion, Transform, Twist, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


def __reorder_output_quaternion(quaternion):
    w, x, y, z = quaternion
    return [x, y, z, w]


def traj_to_msg(traj):
    """Converts the trajectory planned by the planner to a publishable ROS message

    Args:
        traj (gennav.utils.Trajectory): Trajectory planned by the planner

    Returns:
        geometry_msgs/MultiDOFJointTrajectory.msg: A publishable ROS message
    """

    traj_msg = MultiDOFJointTrajectory(points=[], joint_names=[], header=Header())
    for state in traj.path:
        quaternion = __reorder_output_quaternion(
            transforms3d.euler.euler2quat(
                state.orientation.roll,
                state.orientation.pitch,
                state.orientation.yaw,
            )
        )
        velocity = Twist()
        acceleration = Twist()
        print("positions: ", state.position.x, state.position.y, state.position.z)
        # type erroring out
        transforms = Transform(
            translation=Point(x=float(state.position.x), y=float(state.position.y), z=float(state.position.z)),
            rotation=Quaternion(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            ),
        )
        traj_point = MultiDOFJointTrajectoryPoint(
            transforms=[transforms],
            velocities=[velocity],
            accelerations=[acceleration],
            time_from_start=duration.Duration.from_msg(time.Duration(time.time())),
        )
        traj_msg.points.append(traj_point)

    return traj_msg


def msg_to_traj(msg):
    """Converts the trajectory containing ROS message to gennav.utils.Trajectory data type

    Args:
        msg (geometry_msgs/MultiDOFJointTrajectory.msg): The ROS message

    Returns:
        gennav.utils.Trajectory : Object containing the trajectory data
    """

    path = []
    timestamps = []
    for point in msg.points:
        timestamps.append(point.time_from_start)
        position = utils.Point(
            x=point.transforms[0].translation.x,
            y=point.transforms[0].translation.y,
            z=point.transforms[0].translation.z,
        )
        rotation = tf.transformations.euler_from_quaternion(
            [
                point.transforms[0].rotation.x,
                point.transforms[0].rotation.y,
                point.transforms[0].rotation.z,
                point.transforms[0].rotation.w,
            ]
        )
        rotation = utils.OrientationRPY(
            roll=rotation[0], pitch=rotation[1], yaw=rotation[2]
        )
        linear_vel = utils.Vector3D(
            x=point.velocities[0].linear.x,
            y=point.velocities[0].linear.y,
            z=point.velocities[0].linear.z,
        )
        angular_vel = utils.Vector3D(
            x=point.velocities[0].angular.x,
            y=point.velocities[0].angular.y,
            z=point.velocities[0].linear.z,
        )
        velocity = Velocity(linear=linear_vel, angular=angular_vel)
        state = RobotState(position=position, orientation=rotation, velocity=velocity)
        path.append(state)

    traj = Trajectory(path=path, timestamps=timestamps)
    return traj


def Odom_to_RobotState(msg):
    """Converts a ROS message of type nav_msgs/Odometry.msg to an object of type gennav.utils.RobotState

    Args:
        msg (nav_msgs/Odometry.msg): ROS message containing the pose and velocity of the robot

    Returns:
        gennav.utils.RobotState: A RobotState() object which can be passed to the controller
    """

    position = utils.Point(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        z=msg.pose.pose.position.z,
    )

    orientation = tf.transformations.euler_from_quaternion(
        [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
    )
    orientation = utils.OrientationRPY(
        roll=orientation[0], pitch=orientation[1], yaw=orientation[2]
    )

    linear_vel = utils.Vector3D(
        x=msg.twist.twist.linear.x,
        y=msg.twist.twist.linear.y,
        z=msg.twist.twist.linear.z,
    )
    angular_vel = utils.Vector3D(
        x=msg.twist.twist.angular.x,
        y=msg.twist.twist.angular.y,
        z=msg.twist.twist.angular.z,
    )
    velocity = Velocity(linear=linear_vel, angular=angular_vel)

    state = RobotState(position=position, orientation=orientation, velocity=velocity)

    return state


def Velocity_to_Twist(velocity):
    """Converts an object of type gennav.utils.Velocity to a ROS message (Twist) publishable on cmd_vel

    Args:
        velocity (gennav.utils.Velociy): Velocity to be sent to the robot

    Returns:
        geometry_msgs/Twist.msg: ROS message which can be sent to the robot via the cmd_vel topic
    """

    linear = Vector3(x=velocity.linear.x, y=velocity.linear.y, z=velocity.linear.z)
    angular = Vector3(x=velocity.angular.x, y=velocity.angular.y, z=velocity.angular.z)

    msg = Twist(linear=linear, angular=angular)

    return msg


def LaserScan_to_polygons(scan_data, threshold, angle_deviation=0):
    """Converts data of sensor_msgs/LaserScan ROS message type to polygons

    Args:
        scan_data (sensor_msgs.msg.LaserScan): Data to be converted
        threshold (int):Threshold for deciding when to start a new obstacle
        angle_deviation: Deviation of the zero ofthe lidar from zero of the bot(in radians)
    Returns:
        list[list[tuple[float, float, float]]]: Corresponding polygons
    """
    obstacle_1D = [[scan_data.ranges[0]]]
    current_obstacle_index = 0
    for i in range(len(scan_data.ranges) - 1):
        if abs(scan_data.ranges[i] - scan_data.ranges[i + 1]) > threshold:
            obstacle_1D.append([])
            current_obstacle_index += 1
        obstacle_1D[current_obstacle_index].append(scan_data.ranges[i + 1])
    obstacle_list = []
    least_angle = 2 * math.pi / len(scan_data.ranges)
    pt_count = 0

    for i in range(len(obstacle_1D)):
        for j in range(len(obstacle_1D[i])):
            obstacle_1D[i][j] = (
                obstacle_1D[i][j],
                angle_deviation + pt_count * least_angle,
            )
            pt_count = pt_count + 1

    point_list = []
    for obstacle in obstacle_1D:
        for point_rtheta in obstacle:
            point = (
                (point_rtheta[0] * math.cos(point_rtheta[1])),
                (point_rtheta[0] * math.sin(point_rtheta[1])),
                0,
            )
            point_list.append(point)
        obstacle_list.append(point_list)
    return obstacle_list
