import rclpy
import rclpy.logging

from geometry_msgs.msg import PoseStamped
from moveit.planning import  MoveItPy
from grinding_motion_routines.config import MOVEIT_CONFIG

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
