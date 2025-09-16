#!/usr/bin/env python3


import warnings
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp

# constants
from numpy import pi, nan


class MotionGenerator:
    def __init__(
        self, mortar_top_center_position, mortar_inner_size, yaw_twist_limit=[0, 2 * pi]
    ):
        """Supported type : 'Dict' mortar_base_position [x:,y:,z:], 'Dict' mortar_inner_size [x:,y:,z:]"""

        self.mortar_top_center_position = mortar_top_center_position
        self.mortar_inner_size = mortar_inner_size
        self.min_yaw_twist = yaw_twist_limit[0]
        self.max_yaw_twist = yaw_twist_limit[1]

    def _calc_quaternion_of_mortar_inner_wall(
        self, position, angle_scale, yaw_bias, yaw_twist, fixed_quaternion=False
    ):
        """
        乳鉢の内壁に沿った姿勢（クォータニオン）を計算する。
        """
        num_points = position.shape[1]
        t = np.clip(abs(angle_scale), 0.0, 1.0)

        # ----------------------------------------------------------------
        # 1. 基準となるヨー角と、それに基づくワールド座標系での「基準方向」を定義
        # ----------------------------------------------------------------
        base_yaw = np.arctan2(
            self.mortar_top_center_position["y"],
            self.mortar_top_center_position["x"],
        ) + yaw_bias
        
        # このベクトルが、全ての姿勢計算におけるXY平面の向きの基準となる
        ref_x_direction = np.tile([np.cos(base_yaw), np.sin(base_yaw), 0.0], (num_points, 1))

        # ----------------------------------------------------------------
        # 2. 「垂直姿勢」を、基準方向を用いて計算
        # ----------------------------------------------------------------
        z_axis_vert = np.tile([0.0, 0.0, -1.0], (num_points, 1))
        # Y軸 = Z軸 × 基準X方向
        y_axis_vert = np.cross(z_axis_vert, ref_x_direction)
        # X軸 = Y軸 × Z軸 (正規直交系を維持)
        x_axis_vert = np.cross(y_axis_vert, z_axis_vert)
        
        rotations_vertical = Rotation.from_matrix(np.stack([x_axis_vert, y_axis_vert, z_axis_vert], axis=2))

        # ----------------------------------------------------------------
        # 3. 「法線姿勢」を、全く同じロジックで計算
        # ----------------------------------------------------------------
        pos_x, pos_y, pos_z = position[0], position[1], position[2]
        rx2 = (self.mortar_inner_size["x"]) ** 2
        ry2 = (self.mortar_inner_size["y"]) ** 2
        rz2 = (self.mortar_inner_size["z"]) ** 2

        normal_vec = np.stack([
            2 * pos_x / rx2 if rx2 > 0 else np.zeros(num_points),
            2 * pos_y / ry2 if ry2 > 0 else np.zeros(num_points),
            2 * pos_z / rz2 if rz2 > 0 else np.full(num_points, -1.0)
        ], axis=1)
        
        z_axis_normal = normal_vec
        norm = np.linalg.norm(z_axis_normal, axis=1, keepdims=True)
        z_axis_normal = np.divide(z_axis_normal, norm, out=np.zeros_like(z_axis_normal), where=norm!=0)
        z_axis_normal[norm.flatten() == 0] = [0.0, 0.0, -1.0]
        
        # Y軸 = Z軸 × 基準X方向
        y_axis_normal = np.cross(z_axis_normal, ref_x_direction)
        # Z軸と基準X方向が平行になる特異点への対応
        norm = np.linalg.norm(y_axis_normal, axis=1, keepdims=True)
        parallel_indices = (norm < 1e-6).flatten()
        if np.any(parallel_indices):
            # 基準方向をYに変えて再計算
            ref_y_direction = np.tile([-np.sin(base_yaw), np.cos(base_yaw), 0.0], (num_points, 1))
            y_axis_normal[parallel_indices] = np.cross(z_axis_normal[parallel_indices], ref_y_direction[parallel_indices])
            norm = np.linalg.norm(y_axis_normal, axis=1, keepdims=True)

        y_axis_normal = np.divide(y_axis_normal, norm, out=np.zeros_like(y_axis_normal), where=norm!=0)
        
        # X軸 = Y軸 × Z軸
        x_axis_normal = np.cross(y_axis_normal, z_axis_normal)
        
        base_rotations_normal = Rotation.from_matrix(np.stack([x_axis_normal, y_axis_normal, z_axis_normal], axis=2))

        # ----------------------------------------------------------------
        # 4. 意図した追加のねじり（yaw_twist）を適用
        # ----------------------------------------------------------------
        if yaw_twist != 0:
            twist_angles = np.linspace(0, yaw_twist, num_points)
            twist_vectors = z_axis_normal * twist_angles[:, np.newaxis]
            twist_rotation = Rotation.from_rotvec(twist_vectors)
            rotations_normal = twist_rotation * base_rotations_normal
        else:
            rotations_normal = base_rotations_normal
            
        # ----------------------------------------------------------------
        # 5. Slerpで2つの姿勢を補間
        # ----------------------------------------------------------------
        quats = []
        for i in range(num_points):
            rot_v = rotations_vertical[i]
            rot_n = rotations_normal[i]
            
            key_rotations = Rotation.from_quat([rot_v.as_quat(), rot_n.as_quat()])
            slerp = Slerp([0, 1], key_rotations)
            slerp_quat = slerp(t).as_quat()
            quats.append(slerp_quat)
            
        quats = np.array(quats)

        # ----------------------------------------------------------------
        # 6. オプション処理とリターン
        # ----------------------------------------------------------------
        if fixed_quaternion and len(quats) > 0:
            first_quat = quats[0]
            quats = np.tile(first_quat, (len(quats), 1))

        return quats

    def _ellipsoid_z_lower(self, x, y, radius):
        # x^2/rx^2+y^2/ry^2+z^2/rz^2=1より z = -sqrt(rz^2 * (1 - x^2/rx^2 - y^2/ry^2))
        rx, ry, rz = radius[0], radius[1], radius[2]

        # 根号の中が負にならないようにクリップする
        buf = 1 - ((x**2) / (rx**2)) - ((y**2) / (ry**2))
        buf = np.maximum(buf, 0)
        z = -np.sqrt(rz**2 * buf)  # 楕円体の下半分(lower)なのでマイナスつける
        return z

    def _cartesian_to_polar(self, x, y):  # retern 0 <= θ < 2pi
        theta = np.arctan2(y, x)
        r = np.sqrt(x**2 + y**2)

        return r, theta

    def _polar_to_cartesian(self, r, theta):
        x = r * np.cos(theta)
        y = r * np.sin(theta)

        return x, y

    def _lerp_in_cartesian(self, st, ed, points):
        st_x = st[0]
        st_y = st[1]
        ed_x = ed[0]
        ed_y = ed[1]
        dx = abs(ed_x - st_x)
        dy = abs(ed_y - st_y)
        if dx > dy:
            x = np.linspace(st_x, ed_x, points, endpoint=False)
            # st_xとed_xが同じ場合、0除算を避ける
            if ed_x - st_x != 0:
                y = st_y + (ed_y - st_y) * (x - st_x) / (ed_x - st_x)
            else:
                y = np.full_like(x, st_y)
        else:
            y = np.linspace(st_y, ed_y, points, endpoint=False)
            # st_yとed_yが同じ場合、0除算を避ける
            if ed_y - st_y != 0:
                x = st_x + (ed_x - st_x) * (y - st_y) / (ed_y - st_y)
            else:
                x = np.full_like(y, st_x)

        return x, y

    def _lerp_in_polar(self, st, ed, points, number_of_rotations, r_max):
        st_r, st_theta = self._cartesian_to_polar(st[0], st[1])
        ed_r, ed_theta = self._cartesian_to_polar(ed[0], ed[1])

        if st_theta > ed_theta:
            ed_theta += 2 * pi
        ed_theta += (number_of_rotations - 1) * 2 * pi

        dr = abs(ed_r - st_r) / (r_max * 2)
        d_theta = abs(ed_theta - st_theta) / (2 * pi)

        if dr > d_theta:
            r = np.linspace(st_r, ed_r, points, endpoint=False)
            if ed_r - st_r != 0:
                theta = st_theta + (ed_theta - st_theta) * (r - st_r) / (ed_r - st_r)
            else:
                theta = np.linspace(st_theta, ed_theta, points, endpoint=False)
        else:
            theta = np.linspace(st_theta, ed_theta, points, endpoint=False)
            if ed_theta - st_theta != 0:
                r = st_r + (ed_r - st_r) * (theta - st_theta) / (ed_theta - st_theta)
            else:
                r = np.full_like(theta, st_r)
        x = r * np.cos(theta)
        y = r * np.sin(theta)

        return x, y

    def update_fixed_tool_position(self, pos):
        self.mortar_top_center_position = pos

    def create_circular_waypoints(
        self,
        beginning_position,
        end_position,
        beginning_radius_z,
        end_radius_z,
        angle_scale=0,
        yaw_bias=0,
        yaw_twist_per_rotation=0,
        number_of_rotations=1,
        number_of_waypoints_per_circle=10,
        center_position=np.array([0, 0]),
    ):
        """
        supported type
        beginning_position : list [x,y]
        end_position : list [x,y]
        beginning_radius_z : float
        end_radius_z : float
        angle_scale : float
        yaw_bias : float
        yaw_twist_per_rotation : float
        number_of_rotations : int
        number_of_waypoints_per_circle : int
        center_position : list [x,y]
        """
        # chnage unit from mm to m
        beginning_position = np.array(beginning_position).astype(np.float64) * 0.001
        end_position = np.array(end_position).astype(np.float64) * 0.001
        beginning_radius_z = float(beginning_radius_z) * 0.001
        end_radius_z = float(end_radius_z) * 0.001
        circular_center_position = np.array(center_position).astype(np.float64) * 0.001
        total_number_of_waypoints = number_of_rotations * number_of_waypoints_per_circle

        if number_of_rotations < 1:
            raise ValueError(
                "Can't define end θ, you can choose number_of_rotations >= 1"
            )
        elif number_of_waypoints_per_circle < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints_per_circle >= 1"
            )

        if yaw_twist_per_rotation > np.pi:
            warnings.warn(
                "yaw_twist_per_rotation exceeds 180 deg/rot, which may be too fast for most robots and could lead to unexpected behavior."
            )

        # calc twist
        total_yaw_twist = yaw_twist_per_rotation * number_of_rotations

        # Check if total total_yaw_twist exceeds the limit
        if abs(total_yaw_twist) > self.max_yaw_twist:
            limited_yaw_twist = self.max_yaw_twist
            limited_number_of_rotations = int(
                limited_yaw_twist / yaw_twist_per_rotation
            )
            iterations = int(number_of_rotations / limited_number_of_rotations)
            limited_number_of_waypoints = int(total_number_of_waypoints / iterations)
            if limited_number_of_waypoints < 1:
                raise ValueError(
                    "Can't calculate motion, you can choose number_of_waypoints_per_circle >= 1"
                )
            warnings.warn(
                f"Total total_yaw_twist ({total_yaw_twist} rad) exceeds max_yaw_twist ({self.max_yaw_twist} rad). Dividing the motion into {iterations} iterations with limited_yaw_twist ({limited_yaw_twist:.2f} rad)."
            )
            warnings.warn(
                f"limited_number_of_rotations: {limited_number_of_rotations}, limited_number_of_waypoints: {limited_number_of_waypoints}"
            )
            #################### calculate position
            # calc xy
            x, y = self._lerp_in_polar(
                beginning_position,
                end_position,
                limited_number_of_waypoints,
                limited_number_of_rotations,
                self.mortar_inner_size["x"],
            )

            # shift center pos
            x += circular_center_position[0]
            y += circular_center_position[1]

            # check xy in range
            if np.any(np.abs(x) > self.mortar_inner_size["x"]):
                raise ValueError("calculated x is over mortar scale")

            elif np.any(np.abs(y) > self.mortar_inner_size["y"]):
                raise ValueError("calculated y is over mortar scale")

            # calc z
            if end_radius_z < beginning_radius_z:
                raise ValueError(
                    "Calc error: beginning radius z is bigger than end radius z."
                )

            radius_z = np.linspace(
                beginning_radius_z,
                end_radius_z,
                limited_number_of_waypoints,
                endpoint=False,
            )
            z = self._ellipsoid_z_lower(
                x,
                y,
                [self.mortar_inner_size["x"], self.mortar_inner_size["y"], radius_z],
            )

            position = np.array([x, y, z])

            # shift to work pos
            shifted_position = np.array(
                [
                    position[0] + self.mortar_top_center_position["x"],
                    position[1] + self.mortar_top_center_position["y"],
                    position[2] + self.mortar_top_center_position["z"],
                ]
            )

            #################### calculate orientation
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position, angle_scale, yaw_bias, limited_yaw_twist
            )

            #################### create waypoints
            partial_waypoints = np.stack(
                [
                    shifted_position[0],
                    shifted_position[1],
                    shifted_position[2],
                    quat.T[0],
                    quat.T[1],
                    quat.T[2],
                    quat.T[3],
                ]
            ).T
            print(f"partial_waypoints shape: {partial_waypoints.shape}")
            waypoints = []
            for i in range(iterations):
                if i % 2 == 0:
                    waypoints.extend(partial_waypoints[0:-1])
                else:
                    p = partial_waypoints[::-1]
                    waypoints.extend(p[0:-1])

        else:
            #################### calculate position
            # calc xy
            x, y = self._lerp_in_polar(
                beginning_position,
                end_position,
                total_number_of_waypoints,
                number_of_rotations,
                self.mortar_inner_size["x"],
            )

            # shift center pos
            x += circular_center_position[0]
            y += circular_center_position[1]

            # check xy in range
            if np.any(np.abs(x) > self.mortar_inner_size["x"]):
                raise ValueError("calculated x is over mortar scale")

            elif np.any(np.abs(y) > self.mortar_inner_size["y"]):
                raise ValueError("calculated y is over mortar scale")

            # calc z
            if end_radius_z < beginning_radius_z:
                raise ValueError(
                    "Calc error: beginning radius z is bigger than end radius z."
                )

            radius_z = np.linspace(
                beginning_radius_z,
                end_radius_z,
                total_number_of_waypoints,
                endpoint=False,
            )
            z = self._ellipsoid_z_lower(
                x,
                y,
                [self.mortar_inner_size["x"], self.mortar_inner_size["y"], radius_z],
            )

            position = np.array([x, y, z])

            # shift to work pos
            shifted_position = np.array(
                [
                    position[0] + self.mortar_top_center_position["x"],
                    position[1] + self.mortar_top_center_position["y"],
                    position[2] + self.mortar_top_center_position["z"],
                ]
            )

            #################### calculate orientation
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position, angle_scale, yaw_bias, total_yaw_twist
            )

            #################### create waypoints
            waypoints = np.stack(
                [
                    shifted_position[0],
                    shifted_position[1],
                    shifted_position[2],
                    quat.T[0],
                    quat.T[1],
                    quat.T[2],
                    quat.T[3],
                ]
            ).T

        return waypoints

    def create_cartesian_waypoints(
        self,
        beginning_position,
        end_position,
        beginning_radius_z,
        end_radius_z,
        angle_scale=0,
        fixed_quaternion=False,
        yaw_bias=0,
        number_of_waypoints=5,
    ):
        """
        supported type
        beginning_position : list [x,y]
        end_position : list [x,y]
        beginning_radius_z : float
        end_radius_z : float
        angle_scale : float
        fixed_quaternion : bool
        yaw_bias : float
        number_of_waypoints : int
        """
        if number_of_waypoints < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints >= 1"
            )

        # chnage unit from mm to m
        beginning_position = np.array(beginning_position).astype(np.float64) * 0.001
        end_position = np.array(end_position).astype(np.float64) * 0.001
        beginning_radius_z *= 0.001
        end_radius_z *= 0.001

        # calculate position
        x, y = self._lerp_in_cartesian(
            beginning_position, end_position, number_of_waypoints
        )
        radius_z = np.linspace(
            beginning_radius_z, end_radius_z, number_of_waypoints, endpoint=False
        )
        z = self._ellipsoid_z_lower(
            x, y, [self.mortar_inner_size["x"], self.mortar_inner_size["y"], radius_z]
        )
        position = np.array([x, y, z])

        # shift to work pos
        shifted_position = np.array(
            [
                position[0] + self.mortar_top_center_position["x"],
                position[1] + self.mortar_top_center_position["y"],
                position[2] + self.mortar_top_center_position["z"],
            ]
        )

        #################### calculate orientation
        quat = self._calc_quaternion_of_mortar_inner_wall(
            position=position,
            angle_scale=angle_scale,
            yaw_bias=yaw_bias,
            yaw_twist=0,
            fixed_quaternion=fixed_quaternion,
        )

        #################### create waypoints
        waypoints = np.stack(
            [
                shifted_position[0],
                shifted_position[1],
                shifted_position[2],
                quat.T[0],
                quat.T[1],
                quat.T[2],
                quat.T[3],
            ]
        ).T
        # delete duplicated waypoints
        waypoints, index = np.unique(waypoints, axis=0, return_index=True)
        waypoints = waypoints[np.argsort(index)]

        return waypoints

    def create_liner_waypoints_list(
        self,
        beginning_theta,
        end_tehta,
        beginning_length_from_center,
        end_length_from_center,
        beginning_radius_z,
        end_radius_z,
        angle_scale=0,
        fixed_quaternion=False,
        yaw_bias=0,
        number_of_waypoints=5,
        motion_counts=1,
    ):
        """
        supported type
        beginning_theta : float
        end_tehta : float
        beginning_length_from_center : float
        end_length_from_center : float
        beginning_radius_z : float
        end_radius_z : float
        angle_scale : float
        fixed_quaternion : bool
        yaw_bias : float
        number_of_waypoints : int
        motion_counts : int
        """
        if number_of_waypoints < 1:
            raise ValueError(
                "Can't calculate motion, you can choose number_of_waypoints >= 1"
            )

        # chnage unit from mm to m
        beginning_length_from_center *= 0.001
        end_length_from_center *= 0.001
        beginning_radius_z *= 0.001
        end_radius_z *= 0.001

        # calculate position in tahta range
        theta = np.linspace(beginning_theta, end_tehta, motion_counts, endpoint=False)
        beginning_r = np.full_like(theta, beginning_length_from_center)
        end_r = np.full_like(theta, end_length_from_center)
        beginning_x, beginning_y = self._polar_to_cartesian(beginning_r, theta)
        end_x, end_y = self._polar_to_cartesian(end_r, theta)
        radius_z = np.linspace(
            beginning_radius_z, end_radius_z, number_of_waypoints, endpoint=False
        )

        waypoints_list = []
        for bx, by, ex, ey in zip(
            beginning_x, beginning_y, end_x, end_y
        ):
            #################### calculate position
            beginning_position = [bx, by]
            end_position = [ex, ey]
            x, y = self._lerp_in_cartesian(
                beginning_position, end_position, number_of_waypoints
            )
            z = self._ellipsoid_z_lower(
                x, y, [self.mortar_inner_size["x"], self.mortar_inner_size["y"], radius_z]
            )
            position = np.array([x, y, z])

            # shift to work pos
            shifted_position = np.array(
                [
                    position[0] + self.mortar_top_center_position["x"],
                    position[1] + self.mortar_top_center_position["y"],
                    position[2] + self.mortar_top_center_position["z"],
                ]
            )

            #################### calculate orientation
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position=position,
                angle_scale=angle_scale,
                yaw_bias=yaw_bias,
                yaw_twist=0,
                fixed_quaternion=fixed_quaternion,
            )

            #################### create waypoints
            waypoints = np.stack(
                [
                    shifted_position[0],
                    shifted_position[1],
                    shifted_position[2],
                    quat.T[0],
                    quat.T[1],
                    quat.T[2],
                    quat.T[3],
                ]
            ).T
            # delete duplicated waypoints
            waypoints, index = np.unique(waypoints, axis=0, return_index=True)
            waypoints = waypoints[np.argsort(index)]

            #################### append list
            waypoints_list.append(waypoints)

        return waypoints_list

    def create_epicycloid_waypoints(
        self,
        radius_mm,
        ratio_R_r,
        ratio_d_r=1.0,
        waypoints_step_mm=0.5,
        angle_scale=0,
        yaw_bias=0,
        yaw_twist_vel_rad_per_sec=0,
        motion_velocity_mm_per_sec=50.0,
        equidistant_points=True,
    ):
        """
        Create waypoints along an epitrochoid curve.
        An epitrochoid is traced by a point at a distance 'd' from the center of a circle of radius 'r'
        that rolls around the outside of a fixed circle of radius 'R'.
        If d = r, the curve is a standard epicycloid.

        Parameters:
            radius_mm (float): The scale radius in millimeters, defined as R + 2r.
            ratio_R_r (float): The ratio R/r of the fixed circle radius to the rolling circle radius.
            ratio_d_r (float, optional): The ratio d/r of the tracing point distance to the rolling circle radius.
                                          d=r (ratio_d_r=1.0) gives a standard epicycloid.
                                          d<r (ratio_d_r<1.0) gives a curtate epicycloid.
                                          d>r (ratio_d_r>1.0) gives a prolate epicycloid.
                                          Default is 1.0.
            waypoints_step_mm (float, optional): Step size between waypoints in millimeters. Default is 1.0.
            angle_scale (float, optional): Parameter for orientation calculation.
            yaw_bias (float, optional): Yaw bias to be used in orientation calculation.
            yaw_twist_vel_rad_per_sec (float, optional): Yaw twist velocity in radians per second. Default is 0.
            equidistant_points (bool, optional): If True, arranges the points to be equidistant in Cartesian space.
                                                 This may increase computation time. Defaults to False.

        Returns:
            np.ndarray: An array of waypoints, each row containing [x, y, z, qx, qy, qz, qw].
        """
        from fractions import Fraction

        # Convert radius_mm from mm to m
        radius_m = radius_mm * 0.001
        if radius_m <= 0:
            raise ValueError("Scale radius must be greater than 0.")
        if ratio_R_r <= 0:
            raise ValueError("R/r ratio must be greater than 0.")
        if ratio_d_r <= 0:
            raise ValueError("d/r ratio must be greater than 0.")

        # Calculate r, R and d (in meters)
        r = radius_m / (ratio_R_r + 2)
        R = ratio_R_r * r
        d = r * ratio_d_r

        # Determine the complete curve range for theta
        frac = Fraction(ratio_R_r).limit_denominator(1000)
        q = frac.denominator
        theta_max = 2 * np.pi * q

        # First, calculate the total arc length by generating high-density points
        num_oversampled_points = 20000  # High density for accurate arc length calculation
        theta_oversampled = np.linspace(0, theta_max, num_oversampled_points, endpoint=True)
        
        x_oversampled = (R + r) * np.cos(theta_oversampled) - d * np.cos(((R + r) / r) * theta_oversampled)
        y_oversampled = (R + r) * np.sin(theta_oversampled) - d * np.sin(((R + r) / r) * theta_oversampled)

        # Calculate the total arc length
        distances = np.sqrt(np.diff(x_oversampled)**2 + np.diff(y_oversampled)**2)
        total_distance_m = np.sum(distances)
        total_distance_mm = total_distance_m * 1000  # Convert to mm

        # Calculate number of waypoints based on step size
        number_of_waypoints = max(1, int(np.ceil(total_distance_mm / waypoints_step_mm)))

        if equidistant_points:
            # --- Generate equidistant points ---
            # Calculate cumulative arc length
            cumulative_distance = np.insert(np.cumsum(distances), 0, 0)
            
            # Create target distances at specified step intervals
            target_distances = np.linspace(0, cumulative_distance[-1], number_of_waypoints, endpoint=False)
            
            x = np.interp(target_distances, cumulative_distance, x_oversampled)
            y = np.interp(target_distances, cumulative_distance, y_oversampled)

        else:
            # --- Generate points with equidistant theta ---
            theta = np.linspace(0, theta_max, number_of_waypoints, endpoint=False)
            x = (R + r) * np.cos(theta) - d * np.cos(((R + r) / r) * theta)
            y = (R + r) * np.sin(theta) - d * np.sin(((R + r) / r) * theta)

        # --- Common part for both methods ---
        z = self._ellipsoid_z_lower(
            x,
            y,
            [
                self.mortar_inner_size["x"],
                self.mortar_inner_size["y"],
                self.mortar_inner_size["z"],
            ],
        )

        position = np.array([x, y, z])

        # Shift the positions to the working coordinate using mortar_top_center_position
        shifted_position = np.array(
            [
                position[0] + self.mortar_top_center_position["x"],
                position[1] + self.mortar_top_center_position["y"],
                position[2] + self.mortar_top_center_position["z"],
            ]
        )

        # Calculate yaw twist based on velocity and estimated execution time
        total_yaw_twist = 0
        if yaw_twist_vel_rad_per_sec != 0:
            # Estimate total execution time based on distance and motion velocity
            estimated_execution_time = total_distance_mm / motion_velocity_mm_per_sec
            total_yaw_twist = yaw_twist_vel_rad_per_sec * estimated_execution_time
            
            # Check if total yaw twist exceeds the limit
            if abs(total_yaw_twist) > self.max_yaw_twist:
                limited_yaw_twist = self.max_yaw_twist
                # Calculate how many iterations we need
                iterations = int(np.ceil(abs(total_yaw_twist) / self.max_yaw_twist))
                limited_number_of_waypoints = int(number_of_waypoints / iterations)
                
                if limited_number_of_waypoints < 1:
                    raise ValueError(
                        "Can't calculate motion, waypoints per iteration would be less than 1"
                    )
                
                warnings.warn(
                    f"Total yaw_twist ({total_yaw_twist:.3f} rad) exceeds max_yaw_twist ({self.max_yaw_twist} rad). Dividing the motion into {iterations} iterations with limited_yaw_twist ({limited_yaw_twist:.2f} rad)."
                )
                warnings.warn(
                    f"iterations: {iterations}, limited_number_of_waypoints: {limited_number_of_waypoints}"
                )
                
                # Create waypoints for each iteration
                if equidistant_points:
                    # Calculate cumulative arc length for limited waypoints
                    cumulative_distance = np.insert(np.cumsum(distances), 0, 0)
                    target_distances = np.linspace(0, cumulative_distance[-1], limited_number_of_waypoints, endpoint=False)
                    
                    x_limited = np.interp(target_distances, cumulative_distance, x_oversampled)
                    y_limited = np.interp(target_distances, cumulative_distance, y_oversampled)
                else:
                    # Generate limited waypoints with equidistant theta
                    theta_limited = np.linspace(0, theta_max, limited_number_of_waypoints, endpoint=False)
                    x_limited = (R + r) * np.cos(theta_limited) - d * np.cos(((R + r) / r) * theta_limited)
                    y_limited = (R + r) * np.sin(theta_limited) - d * np.sin(((R + r) / r) * theta_limited)
                
                z_limited = self._ellipsoid_z_lower(
                    x_limited,
                    y_limited,
                    [
                        self.mortar_inner_size["x"],
                        self.mortar_inner_size["y"],
                        self.mortar_inner_size["z"],
                    ],
                )
                
                position_limited = np.array([x_limited, y_limited, z_limited])
                
                # Shift to work position
                shifted_position_limited = np.array(
                    [
                        position_limited[0] + self.mortar_top_center_position["x"],
                        position_limited[1] + self.mortar_top_center_position["y"],
                        position_limited[2] + self.mortar_top_center_position["z"],
                    ]
                )
                
                # Calculate orientation for limited waypoints
                quat_limited = self._calc_quaternion_of_mortar_inner_wall(
                    position_limited, angle_scale, yaw_bias, limited_yaw_twist
                )
                
                # Create partial waypoints
                partial_waypoints = np.stack(
                    [
                        shifted_position_limited[0],
                        shifted_position_limited[1],
                        shifted_position_limited[2],
                        quat_limited.T[0],
                        quat_limited.T[1],
                        quat_limited.T[2],
                        quat_limited.T[3],
                    ]
                ).T
                
                print(f"partial_waypoints shape: {partial_waypoints.shape}")
                waypoints = []
                for i in range(iterations):
                    if i % 2 == 0:
                        # Forward direction
                        waypoints.extend(partial_waypoints[0:-1])
                    else:
                        # Reverse direction (yaw folding)
                        p = partial_waypoints[::-1]
                        waypoints.extend(p[0:-1])
                
                waypoints = np.array(waypoints)
                
            else:
                # Normal case: yaw twist within limits
                # Calculate the orientations using the inner wall function.
                quat = self._calc_quaternion_of_mortar_inner_wall(
                    position=position,
                    angle_scale=angle_scale,
                    yaw_bias=yaw_bias,
                    yaw_twist=total_yaw_twist,
                )

                waypoints = np.stack(
                    [
                        shifted_position[0],
                        shifted_position[1],
                        shifted_position[2],
                        quat.T[0],
                        quat.T[1],
                        quat.T[2],
                        quat.T[3],
                    ]
                ).T
                
                # Remove duplicated waypoints and preserve the order
                waypoints, index = np.unique(waypoints, axis=0, return_index=True)
                waypoints = waypoints[np.argsort(index)]
        else:
            # No yaw twist case
            # Calculate the orientations using the inner wall function.
            quat = self._calc_quaternion_of_mortar_inner_wall(
                position=position,
                angle_scale=angle_scale,
                yaw_bias=yaw_bias,
                yaw_twist=0,
            )

            waypoints = np.stack(
                [
                    shifted_position[0],
                    shifted_position[1],
                    shifted_position[2],
                    quat.T[0],
                    quat.T[1],
                    quat.T[2],
                    quat.T[3],
                ]
            ).T
            
            # Remove duplicated waypoints and preserve the order
            waypoints, index = np.unique(waypoints, axis=0, return_index=True)
            waypoints = waypoints[np.argsort(index)]

        print(
            f"Generated {len(waypoints)} waypoints for the epicycloid with radius {radius_mm} mm, R/r ratio {ratio_R_r}, d/r ratio {ratio_d_r}, and step size {waypoints_step_mm} mm."
        )

        return waypoints