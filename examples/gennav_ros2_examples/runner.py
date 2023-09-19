# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from gennav import utils
from gennav.utils import RobotState
from gennav.utils.geometry import Point
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from trajectory_msgs.msg import MultiDOFJointTrajectory
from .commander import Commander
import gennav

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher = self.create_publisher(MultiDOFJointTrajectory , "topic", 10)
        

        general_obstacles_list = [
                          
        [[(8, 5), (7, 8), (2, 9), (3, 5)], [(3, 3), (3, 5), (5, 5), (5, 3)]],
        
        [
            [(2, 10), (7, 10), (7, 1), (6, 1), (6, 6), (4, 6), (4, 9), (2, 9)],
            [(4, 0), (4, 5), (5, 5), (5, 0)],
            [(8, 2), (8, 7), (10, 7), (10, 2)],
        ],
    ]   

        sampler = gennav.utils.samplers.UniformRectSampler(-5, 15, -5, 15) # for uniformly sampling points
        env = gennav.envs.PolygonEnv(bufer_dist = 1.0)
        

        for obstacles in general_obstacles_list: # updating the environment with all obstacles
          env.update(obstacles)
        self.commander = Commander(
            planner=gennav.planners.RRG(sampler=sampler, expand_dis=1.0, max_iter=500),
            env=env,
            msg_dtype=LaserScan,
        )

        # Publisher for planned trajectory
        self.traj_sub = self.create_subscription(
            MultiDOFJointTrajectory, "/gennav/traj", self.traj_cb, 10
        )
        

        start = RobotState(position=Point(0, 0)) # setting start point to (0,0)
        goal = RobotState(position=Point(10, 10))
        self.commander.goto(
            goal=goal, start=start
        )

    def traj_cb(self, msg):
        self.get_logger().info("Publishing: {}".format(msg))
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
