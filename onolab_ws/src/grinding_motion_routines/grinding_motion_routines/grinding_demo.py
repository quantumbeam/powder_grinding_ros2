import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose, Point
from tf_transformations import quaternion_from_euler, quaternion_slerp


from numpy import pi
from math import tau, dist, fabs, cos, sin, sqrt
from cmath import nan
import sys
import copy
import numpy as np

# from ur_pykdl import ur_kinematics
from grinding_motion_routines.helpers import *
from grinding_motion_routines.marker_display import MarkerDisplay


class GrindingWaypoints(Node):
    def __init__(
        self,
        node_name,
        # init_joint_angles_of_grinding,
        # grinding_ready_joint_angles,
        # init_joint_angles_of_gathering,
    ):
        super().__init__(node_name)

        self.declare_parameter("mortar_hight", 0.0)
        self.declare_parameter("mortar_position", [0.0, 0.0, 0.0])
        self.declare_parameter("mortar_scale", [0.0, 0.0, 0.0])

        self.declare_parameter("grinding_pos_begining", [0.0, 0.0])
        self.declare_parameter("grinding_pos_end", [0.0, 0.0])
        self.declare_parameter("grinding_rz_begining", 0.0)
        self.declare_parameter("grinding_rz_end", 0.0)
        self.declare_parameter("grinding_angle_param", 0.0)
        self.declare_parameter("grinding_yaw_angle", 0.0)
        self.declare_parameter("grinding_number_of_rotation", 1)
        self.declare_parameter("grinding_waypoints_counts", 10)
        self.declare_parameter("grinding_center_pos", [0.0, 0.0])

        self.declare_parameter("gathering_theta_begining", 0.0)
        self.declare_parameter("gathering_theta_end", 0.0)
        self.declare_parameter("gathering_length_from_center_begining", 0.0)
        self.declare_parameter("gathering_length_from_center_end", 0.0)
        self.declare_parameter("gathering_rz_begining", 0.0)
        self.declare_parameter("gathering_rz_end", 0.0)
        self.declare_parameter("gathering_angle_param", 0.0)
        self.declare_parameter("gathering_yaw_angle", 0.0)
        self.declare_parameter("gathering_motion_counts", 1)
        self.declare_parameter("gathering_waypoints_counts", 10)
        self.declare_parameter("gathering_center_pos", [0.0, 0.0])
        self.declare_parameter("gathering_fixed_quaternion", False)

        self.mortar_hight = self.get_parameter("mortar_hight").value
        mortar_position = self.get_parameter("mortar_position").value
        self.mortar_position = Point()
        self.mortar_position.x = mortar_position[0]
        self.mortar_position.y = mortar_position[1]
        self.mortar_position.z = mortar_position[2]

        mortar_center_position = Point()
        mortar_center_position = self.mortar_position
        mortar_center_position.z += self.mortar_hight
        self.mortar_center_position = mortar_center_position

        mortar_scale = self.get_parameter("mortar_scale").value
        self.mortar_scale = Point()
        self.mortar_scale.x = mortar_scale[0]
        self.mortar_scale.y = mortar_scale[1]
        self.mortar_scale.z = mortar_scale[2]

        self.grinding_begining_position = self.get_parameter(
            "grinding_pos_begining"
        ).value
        self.grinding_end_position = self.get_parameter("grinding_pos_end").value
        self.grinding_begining_radious_z = self.get_parameter(
            "grinding_rz_begining"
        ).value
        self.grinding_end_radious_z = self.get_parameter("grinding_rz_end").value
        self.grinding_angle_param = self.get_parameter("grinding_angle_param").value
        self.grinding_yaw_angle = self.get_parameter("grinding_yaw_angle").value
        self.grinding_number_of_rotation = self.get_parameter(
            "grinding_number_of_rotation"
        ).value
        self.grinding_waypoints_counts = self.get_parameter(
            "grinding_waypoints_counts"
        ).value
        self.grinding_center_pos = self.get_parameter("grinding_center_pos").value

        self.gathering_theta_begining = self.get_parameter(
            "gathering_theta_begining"
        ).value
        self.gathering_theta_end = self.get_parameter("gathering_theta_end").value
        self.gathering_length_from_center_begining = self.get_parameter(
            "gathering_length_from_center_begining"
        ).value
        self.gathering_length_from_center_end = self.get_parameter(
            "gathering_length_from_center_end"
        ).value
        self.gathering_rz_begining = self.get_parameter("gathering_rz_begining").value
        self.gathering_rz_end = self.get_parameter("gathering_rz_end").value
        self.gathering_angle_param = self.get_parameter("gathering_angle_param").value
        self.gathering_yaw_angle = self.get_parameter("gathering_yaw_angle").value
        self.gathering_motion_counts = self.get_parameter(
            "gathering_motion_counts"
        ).value
        self.gathering_waypoints_counts = self.get_parameter(
            "gathering_waypoints_counts"
        ).value
        self.gathering_fixed_quaternion = self.get_parameter(
            "gathering_fixed_quaternion"
        ).value

        # self.init_joint_angles_of_grinding = init_joint_angles_of_grinding
        # self.grinding_ready_joint_angles = grinding_ready_joint_angles
        # self.init_joint_angles_of_gathering = init_joint_angles_of_gathering

    def calc_quaternion_of_mortar_inner_wall(
        self, position, angle_param, yaw, fixed_quaternion=False
    ):
        quat = []

        pos_x = np.array(position[0])
        pos_y = np.array(position[1])
        pos_z = np.array(position[2])

        #################### calculate orientation
        # angle param < 0  use inverse x,y, mean using inverse slope
        if angle_param < 0:
            pos_x *= -1
            pos_y *= -1

        # normalized position
        norm = np.sqrt(pos_x**2 + pos_y**2 + pos_z**2)
        normalized_pos_x = pos_x / norm
        normalized_pos_y = pos_y / norm
        normalized_pos_z = pos_z / norm

        # rotate xy by the amount of yaw angle
        r, theta = cartesian_to_polar(normalized_pos_x, normalized_pos_y)
        rotated_normalized_pos_x, rotated_normalized_pos_y = polar_to_cartesian(
            r, theta + yaw
        )

        # calc euler of the normal and the verticle to the ground
        roll_of_the_normal = -np.arctan2(rotated_normalized_pos_y, normalized_pos_z)
        pitch_of_the_normal = -np.arctan2(
            rotated_normalized_pos_x,
            np.sqrt(rotated_normalized_pos_y**2 + normalized_pos_z**2),
        )
        yaw_of_the_normal = np.full_like(pos_z, yaw)

        roll_of_the_vertical = np.full_like(pos_x, pi)
        pitch_of_the_vertical = np.full_like(pos_y, 0)
        yaw_of_the_vertical = np.full_like(pos_z, yaw)

        # convert to quaternion
        for r_normal, p_normal, y_normal, r_vertical, p_vertical, y_vertical in zip(
            roll_of_the_normal,
            pitch_of_the_normal,
            yaw_of_the_normal,
            roll_of_the_vertical,
            pitch_of_the_vertical,
            yaw_of_the_vertical,
        ):
            quat_of_the_normal = quaternion_from_euler(r_normal, p_normal, y_normal)
            quat_of_the_vertical = quaternion_from_euler(
                r_vertical, p_vertical, y_vertical
            )

            # slerp by angle param
            slerp_quat = quaternion_slerp(
                quat_of_the_vertical, quat_of_the_normal, fabs(angle_param)
            )
            quat.append(slerp_quat)
        quat = np.array(quat)

        # quatrnionを固定
        if fixed_quaternion:
            quat = np.full_like(quat, quat[0])

        return quat

    def generate_waypoints(self, position, quat):
        pose_array = np.stack(
            [
                position[0],
                position[1],
                position[2],
                quat.T[0],
                quat.T[1],
                quat.T[2],
                quat.T[3],
            ]
        )
        pose_array = pose_array.transpose()

        # delete duplicated waypoints
        pose_array, index = np.unique(pose_array, axis=0, return_index=True)
        pose_array = pose_array[np.argsort(index)]

        # arrayからwaypointsに変換
        pose_buf = Pose()
        waypoints = []
        for pose in pose_array:
            pose_buf.position.x = pose[0]
            pose_buf.position.y = pose[1]
            pose_buf.position.z = pose[2]
            pose_buf.orientation = Quaternion(
                x=pose[3], y=pose[4], z=pose[5], w=pose[6]
            )
            waypoints.append(copy.deepcopy(pose_buf))
        return waypoints

    def create_circular_waypoints(
        self,
        debug=False,
    ):
        #################### get parameters
        begining_position = self.grinding_begining_position
        end_position = self.grinding_end_position
        begining_radious_z = self.grinding_begining_radious_z
        end_radious_z = self.grinding_end_radious_z
        angle_param = self.grinding_angle_param
        yaw_angle = self.grinding_yaw_angle
        number_of_rotations = self.grinding_number_of_rotation
        waypoints_count = self.grinding_waypoints_counts
        center_position = self.grinding_center_pos

        # chnage unit from mm to m
        begining_position = np.array(begining_position).astype(np.float64) * 0.001
        end_position = np.array(end_position).astype(np.float64) * 0.001
        begining_radious_z = float(begining_radious_z) * 0.001
        end_radious_z = float(end_radious_z) * 0.001
        center_position = np.array(center_position).astype(np.float64) * 0.001

        #################### calculate position
        # calc xy
        x, y = lerp_in_polar(
            begining_position,
            end_position,
            waypoints_count,
            number_of_rotations,
            self.mortar_scale.x,
        )

        # shift center pos
        x += center_position[0]
        y += center_position[1]

        # check xy in range
        if np.any(np.abs(x) > self.mortar_scale.x):
            self.get_logger().error("calculated x is over mortar scale")
            return False
        elif np.any(np.abs(y) > self.mortar_scale.y):
            self.get_logger().error("calculated y is over mortar scale")
            return False

        # calc z
        if end_radious_z < begining_radious_z:
            self.get_logger().error(
                "Calc error: begining radious z is bigger than end radious z."
            )
            return False
        radious_z = np.linspace(
            begining_radious_z, end_radious_z, waypoints_count, endpoint=False
        )
        z = ellipsoid_z_lower(
            x, y, [self.mortar_scale.x, self.mortar_scale.y, radious_z]
        )

        position = np.array([x, y, z])

        # shift to work pos
        shifted_position = np.array(
            [
                position[0] + self.mortar_center_position.x,
                position[1] + self.mortar_center_position.y,
                position[2] + self.mortar_center_position.z,
            ]
        )

        #################### calculate orientation
        if angle_param == 0:
            x_for_quat = np.full_like(x, center_position[0])
            y_for_quat = np.full_like(y, center_position[1])
            pos_for_quat = np.array([x_for_quat, y_for_quat, z])
            quat = self.calc_quaternion_of_mortar_inner_wall(
                pos_for_quat, 1.0, yaw_angle
            )
        else:
            quat = self.calc_quaternion_of_mortar_inner_wall(
                position, angle_param, yaw_angle
            )

        #################### convert to waypoints
        waypoints = self.generate_waypoints(shifted_position, quat)

        if debug:
            plot_position_to_debug(shifted_position)

        return waypoints

    def create_liner_waypoints_list(
        self,
        debug=False,
    ):
        #################### get parameters
        begining_theta = self.gathering_theta_begining
        end_tehta = self.gathering_theta_end
        begining_length_from_center = self.gathering_length_from_center_begining
        end_length_from_center = self.gathering_length_from_center_end
        begining_radius_z = self.gathering_rz_begining
        end_radius_z = self.gathering_rz_end
        angle_param = self.gathering_angle_param
        yaw_angle = self.gathering_yaw_angle
        waypoints_count = self.gathering_waypoints_counts
        gathering_motion_count = self.gathering_motion_counts
        fixed_quaternion = (self.gathering_fixed_quaternion,)

        # chnage unit from mm to m
        begining_length_from_center *= 0.001
        end_length_from_center *= 0.001
        begining_radius_z *= 0.001
        end_radius_z *= 0.001

        # calculate position in tahta range
        theta = np.linspace(
            begining_theta, end_tehta, gathering_motion_count, endpoint=False
        )
        begining_r = np.full_like(theta, begining_length_from_center)
        end_r = np.full_like(theta, end_length_from_center)
        begining_x, begining_y = polar_to_cartesian(begining_r, theta)
        end_x, end_y = polar_to_cartesian(end_r, theta)
        waypoints_list = []
        position_array_for_debug = np.array([[nan, nan, nan]]).T
        radius_z = np.linspace(
            begining_radius_z, end_radius_z, waypoints_count, endpoint=False
        )

        for begining_x, begining_y, end_x, end_y in zip(
            begining_x, begining_y, end_x, end_y
        ):
            #################### calculate position
            begining_position = [begining_x, begining_y]
            end_position = [end_x, end_y]
            x, y = lerp_in_cartesian(begining_position, end_position, waypoints_count)
            z = ellipsoid_z_lower(x, y, radius_z)
            position = np.array([x, y, z])

            # shift to work pos
            shifted_position = np.array(
                [
                    position[0] + self.mortar_center_position.x,
                    position[1] + self.mortar_center_position.y,
                    position[2] + self.mortar_center_position.z,
                ]
            )

            #################### calculate orientation
            quat = self.calc_quaternion_of_mortar_inner_wall(
                position=position,
                angle_param=angle_param,
                yaw=yaw_angle,
                fixed_quaternion=fixed_quaternion,
            )

            #################### convert to waypoints
            waypoints = self.generate_waypoints(shifted_position, quat)

            #################### append list
            waypoints_list.append(waypoints)
            position_array_for_debug = np.concatenate(
                [position_array_for_debug, shifted_position], 1
            )

        if debug:
            plot_position_to_debug(position_array_for_debug)

        return waypoints_list


def main():
    # if len(sys.argv) == 1:
    #     print("You need to set xml_file file path to arg1")
    #     exit()
    # else:
    #     URDF_path = sys.argv[1]

    rclpy.init()
    rclpy.create_node("create_waypoints_node")
    # kin = ur_kinematics(URDF_path, base_link="base_link", ee_link="tool0")

    grinding_waypoints = GrindingWaypoints("create_waypoints_node")
    waypoints = grinding_waypoints.create_circular_waypoints(debug=False)
    print(len(waypoints))
    pose_list = [waypoints[i].position for i in range(len(waypoints))]
    for p in pose_list:
        print(p.x, p.y, p.z)

    marker_display = MarkerDisplay("marker_publisher")
    marker_display.display_waypoints(waypoints, scale=0.01)

    rclpy.spin(marker_display)
    grinding_waypoints.destroy_node()
    marker_display.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
