
# #!/usr/bin/env python3

# import rclpy
# import rclpy.logging
# import copy

# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import MotionPlanRequest
# from config import MOVEIT_CONFIG


# class moveItHelper(Node):
#     def __init__(self, node_name="moveIt_helper", controller_name="moveIt", joints=[""]):

#         super().__init__(node_name)
#         self.controller_name = controller_name
#         self.joints = joints

#         if self.joints is None or len(self.joints) == 0:
#             raise Exception('"joints" parameter is not set!')

#         action_topic = "/" + self.controller_name + "/" + "follow_joint_trajectory"
#         self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)
#         self.goals = []
#         self.i = 0



#     ur = MoveItPy(node_name='moveit_py', config_dict=MOVEIT_CONFIG.to_dict())
#     arm = ur.get_planning_component('wrist_3_joint')
#     model = ur.get_robot_model()

#     s1 = arm.get_start_state()
#     print(s1.get_pose('J6'))

#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = 'world'
#     goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
#     goal_pose.pose.position.x += 0.05

#     arm.set_start_to_current_state()
#     arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
#     plan_result = arm.plan()

#     if plan_result:
#         trj = plan_result.trajectory
#         print(trj)
#         ur.execute(trj, controllers=[])
#     else:
#         print('Fail to plan...')
#     return

# def main():lpy
# import rclpy.logging
# from rclpy.node import Node
# from rclpy.action import ActionClient

# import copy
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory
# from control_msgs.action import FollowJointTrajectory
# from moveit.planning import MoveItPy

# from config import MOVEIT_CONFIG 


# class moveItHelper(Node):
#     def __init__(self, node_name="moveIt_helper", controller_name="moveIt", joints=None):
#         super().__init__(node_name)

#         if joints is None or len(joints) == 0:
#             raise Exception('"joints" parameter is not set!')

#         self.controller_name = controller_name
#         self.joints = joints
#         self.logger = self.get_logger()

#         action_topic = f"/{self.controller_name}/follow_joint_trajectory"
#         self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)

#         # MoveItPyの初期化
#         self.ur = MoveItPy(node_name="moveit_py", config_dict=MOVEIT_CONFIG.to_dict())
#         self.arm = self.ur.get_planning_component('wrist_3_joint')
#         self.model = self.ur.get_robot_model()

#     def plan_and_execute(self):
#         s1 = self.arm.get_start_state()
#         self.logger.info(f"Start Pose: {s1.get_pose('J6')}")

#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'world'
#         goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
#         goal_pose.pose.position.x += 0.05

#         self.arm.set_start_to_current_state()
#         self.arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
#         plan_result = self.arm.plan()

#         if plan_result:
#             trj = plan_result.trajectory
#             print(trj)
#             self.ur.execute(trj, controllers=[])
#         else:
#             self.logger.warn("Fail to plan...")

#     def set_waypoints(self, points, duration, send_immediately=False, wait=True):
#         # TODO: 実際に JointTrajectory を生成して ActionClient に送る処理を実装
#         self.logger.info(f"Set waypoints: {points} (duration: {duration}s)")
#         pass


# def main():
#     rclpy.init()
#     moveit_helper = moveItHelper(
#         controller_name="moveIt",
#         joints=[
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint"
#         ]
#     )

#     moveit_helper.plan_and_execute()

#     # もしウェイポイントを送る機能を実装したらこちらを有効化
#     # p1 = [[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]]
#     # p2 = [[2.5, 2.0, 1.5, 1.0, 0.5, 0.0]]
#     # moveit_helper.set_waypoints(p1, 5, send_immediately=True, wait=True)
#     # moveit_helper.set_waypoints(p2, 5, send_immediately=True, wait=True)

#     rclpy.spin(moveit_helper)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#     rclpy.init(args=args)
#     moveit_helper = moveItHelper(
#         controller_name="moveIt"
#         joints=[
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint"  
#         ]
#     )
#     p1 = [[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]]
#     p2 = [[2.5, 2.0, 1.5, 1.0, 0.5, 0.0]]
#     time_to_reach = 5
#     moveit_helper.set_waypoints(p1, time_to_reach, send_immediately=True, wait=True)
#     moveit_helper.set_waypoints(p2, time_to_reach, send_immediately=True, wait=True)
#     rclpy.spin(jtc_helper)

# if __name__ == "__main__":
#     main()


# #!/usr/bin/env python3

# import rclpy
# import rclpy.logging
# from rclpy.node import Node
# from rclpy.action import ActionClient

# import copy
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory
# from control_msgs.action import FollowJointTrajectory
# from moveit.planning import MoveItPy

# from config import MOVEIT_CONFIG 


# class moveItHelper(Node):
#     def __init__(self, node_name="moveIt_helper", controller_name="moveIt", joints=None):
#         super().__init__(node_name)

#         if joints is None or len(joints) == 0:
#             raise Exception('"joints" parameter is not set!')

#         self.controller_name = controller_name
#         self.joints = joints
#         self.logger = self.get_logger()

#         action_topic = f"/{self.controller_name}/follow_joint_trajectory"
#         self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)

#         # MoveItPyの初期化
#         self.ur = MoveItPy(node_name="moveit_py", config_dict=MOVEIT_CONFIG.to_dict())
#         self.arm = self.ur.get_planning_component('wrist_3_joint')
#         self.model = self.ur.get_robot_model()

#     def plan_and_execute(self):
#         s1 = self.arm.get_start_state()
#         self.logger.info(f"Start Pose: {s1.get_pose('J6')}")

#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'world'
#         goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
#         goal_pose.pose.position.x += 0.05

#         self.arm.set_start_to_current_state()
#         self.arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
#         plan_result = self.arm.plan()

#         if plan_result:
#             trj = plan_result.trajectory
#             print(trj)
#             self.ur.execute(trj, controllers=[])
#         else:
#             self.logger.warn("Fail to plan...")

#     def set_waypoints(self, points, duration, send_immediately=False, wait=True):
#         # TODO: 実際に JointTrajectory を生成して ActionClient に送る処理を実装
#         self.logger.info(f"Set waypoints: {points} (duration: {duration}s)")
#         pass


# def main():
#     rclpy.init()
#     moveit_helper = moveItHelper(
#         controller_name="moveIt",
#         joints=[
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint"
#         ]
#     )
#!/usr/bin/env python3

# import rclpy
# import rclpy.logging
# from rclpy.node import Node
# from rclpy.action import ActionClient

# import copy
# from geometry_msgs.msg import PoseStamped
# from moveit_msgs.msg import MotionPlanRequest, RobotTrajectory
# from control_msgs.action import FollowJointTrajectory
# from moveit.planning import MoveItPy

# from config import MOVEIT_CONFIG 


# class moveItHelper(Node):
#     def __init__(self, node_name="moveIt_helper", controller_name="moveIt", joints=None):
#         super().__init__(node_name)

#         if joints is None or len(joints) == 0:
#             raise Exception('"joints" parameter is not set!')

#         self.controller_name = controller_name
#         self.joints = joints
#         self.logger = self.get_logger()

#         action_topic = f"/{self.controller_name}/follow_joint_trajectory"
#         self.action_client = ActionClient(self, FollowJointTrajectory, action_topic)

#         # MoveItPyの初期化
#         self.ur = MoveItPy(node_name="moveit_py", config_dict=MOVEIT_CONFIG.to_dict())
#         self.arm = self.ur.get_planning_component('wrist_3_joint')
#         self.model = self.ur.get_robot_model()

#     def plan_and_execute(self):
#         s1 = self.arm.get_start_state()
#         self.logger.info(f"Start Pose: {s1.get_pose('J6')}")

#         goal_pose = PoseStamped()
#         goal_pose.header.frame_id = 'world'
#         goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
#         goal_pose.pose.position.x += 0.05

#         self.arm.set_start_to_current_state()
#         self.arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
#         plan_result = self.arm.plan()

#         if plan_result:
#             trj = plan_result.trajectory
#             print(trj)
#             self.ur.execute(trj, controllers=[])
#         else:
#             self.logger.warn("Fail to plan...")

#     def set_waypoints(self, points, duration, send_immediately=False, wait=True):
#         # TODO: 実際に JointTrajectory を生成して ActionClient に送る処理を実装
#         self.logger.info(f"Set waypoints: {points} (duration: {duration}s)")
#         pass


# def main():
#     rclpy.init()
#     moveit_helper = moveItHelper(
#         controller_name="moveIt",
#         joints=[
#             "shoulder_pan_joint",
#             "shoulder_lift_joint",
#             "elbow_joint",
#             "wrist_1_joint",
#             "wrist_2_joint",
#             "wrist_3_joint"
#         ]
#     )

#     moveit_helper.plan_and_execute()

#     # もしウェイポイントを送る機能を実装したらこちらを有効化
#     # p1 = [[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]]
#     # p2 = [[2.5, 2.0, 1.5, 1.0, 0.5, 0.0]]
#     # moveit_helper.set_waypoints(p1, 5, send_immediately=True, wait=True)
#     # moveit_helper.set_waypoints(p2, 5, send_immediately=True, wait=True)

#     rclpy.spin(moveit_helper)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


#     moveit_helper.plan_and_execute()

#     # もしウェイポイントを送る機能を実装したらこちらを有効化
#     # p1 = [[0.0, 0.5, 1.0, 1.5, 2.0, 2.5]]
#     # p2 = [[2.5, 2.0, 1.5, 1.0, 0.5, 0.0]]
#     # moveit_helper.set_waypoints(p1, 5, send_immediately=True, wait=True)
#     # moveit_helper.set_waypoints(p2, 5, send_immediately=True, wait=True)

#     rclpy.spin(moveit_helper)
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()


import rclpy
import rclpy.logging

from geometry_msgs.msg import PoseStamped
from moveit.planning import  MoveItPy
from config import MOVEIT_CONFIG

def main():
    rclpy.init()
    logger = rclpy.logging.get_logger('moveit_py.pose_goal')
    
    cobotta = MoveItPy(node_name='moveit_py', config_dict=MOVEIT_CONFIG.to_dict())
    arm = cobotta.get_planning_component('arm')
    logger.info("MoveItPy instance created")
    
    model = cobotta.get_robot_model()
    
    s1=arm.get_start_state()
    print(s1.get_pose('J6'))
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'world'
    goal_pose.pose = copy.deepcopy(s1.get_pose('J6'))
    goal_pose.pose.position.x += 0.05
    
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link='J6')
    plan_result=arm.plan()
    if plan_result:
        trj = plan_result.trajectory
        print(trj)
        cobotta.execute(trj, controllers=[])
    else:
        print('Fail to plan...')
    
    cobotta.shutdown()
