import rclpy
from rclpy.node import Node
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import yaml

class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__('planning_scene_loader')
        self.declare_parameter('move_group_name', 'ur_manipulator')
        self.declare_parameter('table_position', [0.0, 0.0, 0.0])
        self.declare_parameter('table_scale', [0.9, 1.5, 0.1])
        self.declare_parameter('mortar_top_position', [0.39436323098562737, 0.3668537939641375, 0.04253480790823799])
        self.declare_parameter('mortar_inner_scale', [0.04, 0.04, 0.035])

        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.table_position = self.get_parameter('table_position').get_parameter_value().double_array_value
        self.table_scale = self.get_parameter('table_scale').get_parameter_value().double_array_value
        self.mortar_top_position = self.get_parameter('mortar_top_position').get_parameter_value().double_array_value
        self.mortar_inner_scale = self.get_parameter('mortar_inner_scale').get_parameter_value().double_array_value

        self.scene = PlanningSceneInterface()

        self.add_table()
        self.add_mortar()

    def add_table(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = 'world'
        table_pose.pose.position.x = self.table_position[0]
        table_pose.pose.position.y = self.table_position[1]
        table_pose.pose.position.z = self.table_position[2]
        self.scene.add_box('table', table_pose, size=(self.table_scale[0], self.table_scale[1], self.table_scale[2]))

    def add_mortar(self):
        mortar_pose = PoseStamped()
        mortar_pose.header.frame_id = 'world'
        mortar_pose.pose.position.x = self.mortar_top_position[0]
        mortar_pose.pose.position.y = self.mortar_top_position[1]
        mortar_pose.pose.position.z = self.mortar_top_position[2]
        self.scene.add_box('mortar', mortar_pose, size=(self.mortar_inner_scale[0], self.mortar_inner_scale[1], self.mortar_inner_scale[2]))

def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()