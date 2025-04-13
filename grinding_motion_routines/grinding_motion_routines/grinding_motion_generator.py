#!/usr/bin/env python3

import warnings
from typing import List, Dict, Tuple, Optional, Union
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# constants
from numpy import pi, nan

MM_TO_M = 0.001

# --- Helper Functions (previously static methods) ---

def cartesian_to_polar(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Converts Cartesian coordinates to polar coordinates."""
    radius = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)  # Returns angle in range [-pi, pi]
    return radius, theta

def polar_to_cartesian(radius: np.ndarray, theta: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Converts polar coordinates to Cartesian coordinates."""
    x = radius * np.cos(theta)
    y = radius * np.sin(theta)
    return x, y

def lerp_in_cartesian(
    start_pos: np.ndarray, end_pos: np.ndarray, num_points: int
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Linear interpolation between two Cartesian points (exclusive of endpoint).

    Args:
        start_pos: Starting [x, y] position.
        end_pos: Ending [x, y] position.
        num_points: Number of points to generate.

    Returns:
        Tuple of interpolated x and y coordinate arrays.
    """
    start_pos = np.asarray(start_pos)
    end_pos = np.asarray(end_pos)
    # Use np.linspace for direct interpolation, endpoint=False
    x = np.linspace(start_pos[0], end_pos[0], num_points, endpoint=False)
    y = np.linspace(start_pos[1], end_pos[1], num_points, endpoint=False)
    return x, y

def lerp_in_polar(
    start_pos_xy: np.ndarray,
    end_pos_xy: np.ndarray,
    num_points: int,
    num_rotations: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Linear interpolation between two points defined in polar coordinates,
    spanning multiple rotations (exclusive of endpoint).

    Args:
        start_pos_xy: Starting [x, y] position.
        end_pos_xy: Ending [x, y] position.
        num_points: Number of points to generate.
        num_rotations: Number of full rotations to span during interpolation.

    Returns:
        Tuple of interpolated x and y coordinate arrays in Cartesian space.
    """
    start_pos_xy = np.asarray(start_pos_xy)
    end_pos_xy = np.asarray(end_pos_xy)

    if num_rotations < 1:
        warnings.warn("num_rotations < 1 may lead to unexpected interpolation paths. Treating as single rotation segment.")
        num_rotations = 1

    start_r, start_theta = cartesian_to_polar(start_pos_xy[0], start_pos_xy[1])
    end_r, end_theta = cartesian_to_polar(end_pos_xy[0], end_pos_xy[1])

    # Adjust end_theta for multiple rotations and shortest path wrapping
    delta_theta = end_theta - start_theta
    # Adjust for shortest path if only one rotation and crossing +/- pi boundary
    if num_rotations == 1:
        if delta_theta > pi:
            end_theta -= 2 * pi
        elif delta_theta < -pi:
            end_theta += 2 * pi
    # Add full rotations (adjust end_theta first if needed)
    end_theta += (num_rotations -1) * 2 * pi # Add full rotations after potential wrap adjustment


    # Interpolate radius and theta linearly using linspace (endpoint=False)
    r_interp = np.linspace(start_r, end_r, num_points, endpoint=False)
    theta_interp = np.linspace(start_theta, end_theta, num_points, endpoint=False)

    x, y = polar_to_cartesian(r_interp, theta_interp)
    return x, y

# --- GrindingMotionGenerator Class ---

class GrindingMotionGenerator:
    """
    Generates waypoints for grinding motions inside a mortar modeled as a lower ellipsoid half.

    Attributes:
        mortar_top_center_position (Dict[str, float]): The [x, y, z] position of the mortar's top center in meters.
        mortar_inner_radii (Dict[str, float]): The inner radii [rx, ry, rz] of the mortar ellipsoid in meters.
        yaw_twist_limit (Tuple[float, float]): Min and max allowed absolute yaw twist angle in radians.
    """
    def __init__(
        self,
        mortar_top_center_position: Dict[str, float],
        mortar_inner_size_mm: Dict[str, float], # Expect size (diameter/depth) in mm
        yaw_twist_limit: Tuple[float, float] = (0, 2 * pi)
    ):
        """
        Initializes the GrindingMotionGenerator.

        Args:
            mortar_top_center_position: The [x, y, z] position of the mortar's top center (in meters).
            mortar_inner_size_mm: The inner dimensions [x, y, z] of the mortar (in mm),
                                  representing the diameters or depth. These are converted to radii in meters.
            yaw_twist_limit: Min and max allowed absolute yaw twist angle (in radians).
        """
        if not all(key in mortar_top_center_position for key in ['x', 'y', 'z']):
            raise ValueError("mortar_top_center_position must contain 'x', 'y', 'z' keys.")
        if not all(key in mortar_inner_size_mm for key in ['x', 'y', 'z']):
            raise ValueError("mortar_inner_size_mm must contain 'x', 'y', 'z' keys.")
        if not all(mortar_inner_size_mm[key] > 0 for key in ['x', 'y', 'z']):
             raise ValueError("Mortar inner dimensions must be positive.")

        self.mortar_top_center_position = mortar_top_center_position
        # Convert diameters/depth (size) in mm to radii in meters
        self.mortar_inner_radii = {
            "x": mortar_inner_size_mm["x"] * MM_TO_M / 2.0,
            "y": mortar_inner_size_mm["y"] * MM_TO_M / 2.0,
            "z": mortar_inner_size_mm["z"] * MM_TO_M / 2.0, # Assuming z is depth/diameter -> radius
        }
        # Store limits on absolute twist
        self.min_abs_yaw_twist = abs(yaw_twist_limit[0])
        self.max_abs_yaw_twist = abs(yaw_twist_limit[1])
        if self.min_abs_yaw_twist > self.max_abs_yaw_twist:
             raise ValueError("Yaw twist limit min cannot be greater than max.")


    def update_mortar_position(self, pos: Dict[str, float]):
        """Updates the mortar's top center position."""
        if not all(key in pos for key in ['x', 'y', 'z']):
            raise ValueError("Position must contain 'x', 'y', 'z' keys.")
        self.mortar_top_center_position = pos

    def _calculate_ellipsoid_z(self, x: np.ndarray, y: np.ndarray, radius_z: Union[float, np.ndarray]) -> np.ndarray:
        """
        Calculates the z-coordinate on the lower half of the mortar's inner ellipsoid surface.

        Equation: x²/rx² + y²/ry² + z²/rz² = 1
        Solving for z (lower half): z = -sqrt(rz² * (1 - x²/rx² - y²/ry²))

        Args:
            x: X-coordinates relative to the mortar center (in meters).
            y: Y-coordinates relative to the mortar center (in meters).
            radius_z: The z-radius (or radii) of the ellipsoid at the given points (in meters).

        Returns:
            The corresponding z-coordinates on the lower ellipsoid surface (in meters).

        Raises:
            ValueError: If a point (x, y) is outside the ellipsoid projection for the given radii.
        """
        rx = self.mortar_inner_radii["x"]
        ry = self.mortar_inner_radii["y"]
        # Ensure inputs are numpy arrays for vectorized operations
        x = np.asarray(x)
        y = np.asarray(y)
        rz = np.asarray(radius_z) # Can be scalar or array

        # Check for division by zero if radii are zero (should be caught in init)
        if rx <= 0 or ry <= 0:
            raise ValueError("Mortar x/y radii must be positive.")

        term_inside_sqrt = 1.0 - (x**2 / rx**2) - (y**2 / ry**2)

        # Check if any point is outside the ellipsoid projection (allowing for small tolerance)
        if np.any(term_inside_sqrt < -1e-9): # Allow slightly negative due to float errors
            problematic_indices = np.where(term_inside_sqrt < -1e-9)[0]
            max_val = np.max((x[problematic_indices]**2 / rx**2) + (y[problematic_indices]**2 / ry**2))
            raise ValueError(
                f"Point(s) at indices {problematic_indices} are outside the ellipsoid projection "
                f"(max x²/rx² + y²/ry² = {max_val:.4f} > 1)."
            )
        # Ensure term is non-negative before sqrt
        term_inside_sqrt = np.maximum(0, term_inside_sqrt)

        # Check for division by zero if rz is zero
        if np.any(rz <= 0):
             # If rz is an array, find where it's non-positive
             if isinstance(rz, np.ndarray):
                 zero_rz_indices = np.where(rz <=0)[0]
                 raise ValueError(f"Ellipsoid z-radius must be positive. Found non-positive value at indices: {zero_rz_indices}")
             else: # rz is scalar
                 raise ValueError("Ellipsoid z-radius must be positive.")


        z = -np.sqrt(rz**2 * term_inside_sqrt) # Lower half -> negative sqrt
        return z

    def _calculate_orientation(
        self,
        position_relative: np.ndarray, # Shape (3, N) - x, y, z relative to mortar center
        radius_z_interp: np.ndarray,   # Shape (N,) - z-radius for each point
        angle_scale: float,
        yaw_bias: Optional[float],
        yaw_twist_total: float,
        fixed_quaternion: bool = False
    ) -> np.ndarray:
        """
        Calculates the orientation (quaternion) for each waypoint.

        The orientation interpolates between pointing vertically downwards and
        pointing normal to the inner mortar surface, with optional yaw control.

        Args:
            position_relative: Waypoint positions relative to mortar center (shape [3, N], meters).
            radius_z_interp: Z-radius of the ellipsoid for each waypoint (shape [N,], meters).
            angle_scale: Interpolation factor (0=vertical, 1=normal, <0=inverted normal).
            yaw_bias: Fixed yaw offset (radians). Applied if yaw_twist_total is 0.
            yaw_twist_total: Total yaw rotation over the trajectory (radians).
            fixed_quaternion: If True, use the orientation of the first waypoint for all points.

        Returns:
            Array of quaternions (shape [N, 4]).
        """
        num_waypoints = position_relative.shape[1]
        if num_waypoints == 0:
            return np.empty((0, 4))

        pos_x, pos_y, pos_z = position_relative # Each is shape (N,)

        # --- Calculate Target Yaw ---
        if yaw_twist_total != 0:
            abs_twist = abs(yaw_twist_total)
            if not (self.min_abs_yaw_twist <= abs_twist <= self.max_abs_yaw_twist):
                 warnings.warn(
                     f"Total absolute yaw twist {abs_twist:.2f} rad is outside the limit "
                     f"[{self.min_abs_yaw_twist:.2f}, {self.max_abs_yaw_twist:.2f}] rad."
                 )
            # Linear twist from start_yaw to end_yaw. Assume twist is applied relative to initial orientation.
            # Let's assume the twist is applied linearly over the path.
            # If yaw_bias is also given, should it be the starting yaw? Assume twist overrides bias.
            start_yaw = yaw_bias if yaw_bias is not None else 0.0 # Start yaw if no twist specified
            end_yaw = start_yaw + yaw_twist_total
            yaw_values = np.linspace(start_yaw, end_yaw, num_waypoints)

        elif yaw_bias is not None:
            yaw_values = np.full(num_waypoints, yaw_bias)
        else:
            yaw_values = np.zeros(num_waypoints) # Default to zero yaw offset

        # --- Calculate Normal Vector Orientation ---
        # Calculate surface normal (gradient of ellipsoid function F = x²/rx² + y²/ry² + z²/rz² - 1 = 0)
        # grad(F) = [2x/rx², 2y/ry², 2z/rz²]
        # Normal vector points outwards. For inner surface, we need the negative gradient (-grad(F)).
        rx2 = self.mortar_inner_radii["x"]**2
        ry2 = self.mortar_inner_radii["y"]**2
        rz2 = radius_z_interp**2 # Use interpolated z-radius for each point

        # Avoid division by zero if radii are zero (should be caught earlier)
        if rx2 <= 0 or ry2 <= 0 or np.any(rz2 <= 0):
             raise ValueError("Invalid zero or negative squared radii detected during normal calculation.")

        normal_x = -2 * pos_x / rx2 # Components of inward normal
        normal_y = -2 * pos_y / ry2
        normal_z = -2 * pos_z / rz2

        # Normalize the normal vector
        norm_magnitude = np.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
        # Handle points potentially at the center (0,0,0) or where norm is very small
        valid_norm_mask = norm_magnitude > 1e-9
        inward_normal = np.zeros_like(position_relative) # Initialize shape (3, N)

        inward_normal[0, valid_norm_mask] = normal_x[valid_norm_mask] / norm_magnitude[valid_norm_mask]
        inward_normal[1, valid_norm_mask] = normal_y[valid_norm_mask] / norm_magnitude[valid_norm_mask]
        inward_normal[2, valid_norm_mask] = normal_z[valid_norm_mask] / norm_magnitude[valid_norm_mask]

        # For points with near-zero norm (e.g., center), default normal to pointing straight up (inward -> [0,0,1])
        # This corresponds to the tool pointing straight down.
        inward_normal[0, ~valid_norm_mask] = 0
        inward_normal[1, ~valid_norm_mask] = 0
        inward_normal[2, ~valid_norm_mask] = 1 # Pointing up (tool Z will point down)


        # --- Calculate Orientation from Normal (Tool Z aligned with -Normal) ---
        # We want the tool's Z-axis ([0, 0, 1] in tool frame) to align with the *negative* inward normal vector (pointing into surface).
        target_z_axis = -inward_normal # Shape (3, N)

        # Calculate Roll/Pitch to align Z-axis with the target_z_axis
        # Using 'xyz' Euler sequence convention (Roll around X, Pitch around Y, Yaw around Z)
        # Pitch (rotation around Y): asin(-target_z_axis_x)
        # Roll (rotation around X): atan2(-target_z_axis_y, -target_z_axis_z)
        pitch_normal = np.arcsin(-target_z_axis[0, :])
        roll_normal = np.arctan2(-target_z_axis[1, :], -target_z_axis[2, :])

        # Create rotation objects for normal alignment (without yaw)
        rot_normal = Rotation.from_euler('xyz', np.stack([roll_normal, pitch_normal, np.zeros_like(roll_normal)], axis=1))

        # --- Calculate Vertical Orientation (Tool Z pointing down [-world_z]) ---
        # Euler angles 'xyz': [pi, 0, 0] means Roll=pi, Pitch=0, Yaw=0
        rot_vertical = Rotation.from_euler('xyz', [pi, 0, 0]) # Single rotation object

        # --- Interpolate using Slerp ---
        interp_scale = np.clip(abs(angle_scale), 0, 1) # Ensure scale is [0, 1]

        # Handle angle_scale < 0 (target is inverted normal)
        if angle_scale < 0:
             # Target Z axis becomes +inward_normal
             target_z_axis_inv = inward_normal
             pitch_normal_inv = np.arcsin(-target_z_axis_inv[0, :])
             roll_normal_inv = np.arctan2(-target_z_axis_inv[1, :], -target_z_axis_inv[2, :])
             rot_normal = Rotation.from_euler('xyz', np.stack([roll_normal_inv, pitch_normal_inv, np.zeros_like(roll_normal_inv)], axis=1))

        # Perform Slerp for each waypoint
        key_times = [0, 1]
        quat_interp = np.zeros((num_waypoints, 4))

        quat_vertical = rot_vertical.as_quat() # Shape (4,)
        quats_normal = rot_normal.as_quat()    # Shape (N, 4)

        # Check if quats_normal has the expected shape
        if quats_normal.shape != (num_waypoints, 4):
             raise RuntimeError(f"Unexpected shape for normal quaternions: {quats_normal.shape}")


        for i in range(num_waypoints):
            # Slerp between the single vertical quaternion and the i-th normal quaternion
            slerp_rotations = Rotation.from_quat([quat_vertical, quats_normal[i]])
            slerp = Slerp(key_times, slerp_rotations)
            quat_interp[i] = slerp(interp_scale).as_quat()

        # --- Apply Yaw ---
        # Create yaw rotations (around Z axis) and apply them *after* the roll/pitch alignment
        yaw_rotations = Rotation.from_euler('z', yaw_values)
        final_rotations = yaw_rotations * Rotation.from_quat(quat_interp) # Apply yaw

        final_quats = final_rotations.as_quat()

        # --- Handle Fixed Quaternion ---
        if fixed_quaternion and num_waypoints > 0:
            final_quats = np.tile(final_quats[0], (num_waypoints, 1)) # Repeat first quat

        return final_quats # Shape (N, 4)


    def _create_waypoints_array(self, positions_global: np.ndarray, orientations_quat: np.ndarray) -> np.ndarray:
        """
        Combines global positions and orientations into the final waypoint array.

        Args:
            positions_global: Global waypoint positions (shape [3, N], meters).
            orientations_quat: Waypoint orientations as quaternions (shape [N, 4]).

        Returns:
            Waypoint array (shape [N, 7] -> x, y, z, qx, qy, qz, qw).
        """
        num_pos = positions_global.shape[1]
        num_ori = orientations_quat.shape[0]

        if num_pos != num_ori:
            raise ValueError(f"Number of positions ({num_pos}) and orientations ({num_ori}) must match.")
        if num_pos == 0:
            return np.empty((0, 7))

        # Ensure positions are (N, 3) and orientations are (N, 4) for hstack
        if positions_global.shape[0] != 3:
             raise ValueError(f"Expected positions_global shape (3, N), got {positions_global.shape}")
        if orientations_quat.shape[1] != 4:
             raise ValueError(f"Expected orientations_quat shape (N, 4), got {orientations_quat.shape}")


        waypoints = np.hstack((positions_global.T, orientations_quat)) # Combine pos (N, 3) and quat (N, 4)

        # Optional: Remove consecutive duplicated waypoints (more robust than unique)
        if waypoints.shape[0] > 1:
            diff = np.diff(waypoints, axis=0)
            mask = np.append(True, np.any(np.abs(diff) > 1e-9, axis=1)) # Keep first and non-duplicates
            waypoints = waypoints[mask]

        return waypoints

    def create_circular_waypoints(
        self,
        beginning_position_mm: List[float], # [x, y] relative to center_position_mm (mm)
        end_position_mm: List[float],       # [x, y] relative to center_position_mm (mm)
        # Make z-radii optional, default to None
        beginning_radius_z_mm: Optional[float] = None, # Ellipsoid z-radius at start (mm). Defaults to mortar's inner z-radius if None.
        end_radius_z_mm: Optional[float] = None,       # Ellipsoid z-radius at end (mm). Defaults to mortar's inner z-radius if None.
        angle_scale: float = 0.5,
        yaw_bias: Optional[float] = None,
        yaw_twist_per_rotation: float = 0,
        number_of_rotations: int = 1,
        number_of_waypoints_per_circle: int = 20,
        center_position_mm: List[float] = [0.0, 0.0], # [x, y] offset from mortar center (mm)
    ) -> np.ndarray:
        """
        Generates waypoints following a circular/spiral path on the mortar surface.

        Args:
            beginning_position_mm: Start XY position relative to center_position_mm (mm).
            end_position_mm: End XY position relative to center_position_mm (mm).
            beginning_radius_z_mm: Ellipsoid Z-radius at the start (mm). Defaults to mortar's inner z-radius if None.
            end_radius_z_mm: Ellipsoid Z-radius at the end (mm). Defaults to mortar's inner z-radius if None.
            angle_scale: Tool orientation interpolation factor (0=vertical, 1=normal).
            yaw_bias: Fixed yaw offset (radians). Used if yaw_twist_per_rotation is 0.
            yaw_twist_per_rotation: Yaw change per full rotation (radians).
            number_of_rotations: Total number of full rotations in the spiral.
            number_of_waypoints_per_circle: Resolution of the path.
            center_position_mm: XY offset of the circular path center relative to mortar center (mm).

        Returns:
            Array of waypoints (shape [N, 7]).

        Raises:
            ValueError: If input parameters are invalid or path goes outside mortar bounds.
        """
        if number_of_rotations < 1:
            raise ValueError("number_of_rotations must be >= 1")
        if number_of_waypoints_per_circle < 1:
            raise ValueError("number_of_waypoints_per_circle must be >= 1")
        if abs(yaw_twist_per_rotation) > pi:
            warnings.warn(
                f"Absolute yaw_twist_per_rotation ({abs(yaw_twist_per_rotation):.2f} rad) exceeds pi (180 deg/rot)."
            )

        # --- Set default Z radii if not provided ---
        default_rz_m = self.mortar_inner_radii["z"]
        default_rz_mm = default_rz_m / MM_TO_M
        start_rz_mm_to_use = beginning_radius_z_mm if beginning_radius_z_mm is not None else default_rz_mm
        end_rz_mm_to_use = end_radius_z_mm if end_radius_z_mm is not None else default_rz_mm

        # --- Check if end radius is smaller than beginning radius (after defaults) ---
        if end_rz_mm_to_use < start_rz_mm_to_use:
             if not np.isclose(end_rz_mm_to_use, start_rz_mm_to_use):
                 raise ValueError(f"Effective end_radius_z_mm ({end_rz_mm_to_use:.2f}) cannot be smaller than effective beginning_radius_z_mm ({start_rz_mm_to_use:.2f}).")

        # --- Convert inputs to meters and numpy arrays ---
        start_pos_xy_offset = np.array(beginning_position_mm) * MM_TO_M
        end_pos_xy_offset = np.array(end_position_mm) * MM_TO_M
        # Use the determined mm values (provided or default) and convert to meters
        start_rz = start_rz_mm_to_use * MM_TO_M
        end_rz = end_rz_mm_to_use * MM_TO_M
        center_offset_xy = np.array(center_position_mm) * MM_TO_M

        total_number_of_waypoints = number_of_rotations * number_of_waypoints_per_circle
        if total_number_of_waypoints == 0:
             return np.empty((0, 7))

        # --- Calculate XY positions (relative to center_offset_xy) ---
        x_rel_offset, y_rel_offset = lerp_in_polar(
            start_pos_xy_offset,
            end_pos_xy_offset,
            total_number_of_waypoints,
            number_of_rotations,
        )

        # --- Calculate XY positions (relative to mortar center) ---
        x_rel_mortar = x_rel_offset + center_offset_xy[0]
        y_rel_mortar = y_rel_offset + center_offset_xy[1]

        # --- Check if path exceeds mortar XY boundaries ---
        max_rx = self.mortar_inner_radii["x"]
        max_ry = self.mortar_inner_radii["y"]
        if np.any(np.abs(x_rel_mortar) > max_rx + 1e-6): # Add tolerance
             exceeding_indices = np.where(np.abs(x_rel_mortar) > max_rx + 1e-6)[0]
             raise ValueError(f"Calculated X path exceeds mortar radius ({max_rx*1000:.1f} mm) at indices {exceeding_indices}.")
        if np.any(np.abs(y_rel_mortar) > max_ry + 1e-6): # Add tolerance
             exceeding_indices = np.where(np.abs(y_rel_mortar) > max_ry + 1e-6)[0]
             raise ValueError(f"Calculated Y path exceeds mortar radius ({max_ry*1000:.1f} mm) at indices {exceeding_indices}.")

        # --- Calculate Z positions (relative to mortar center) ---
        # Interpolate the z-radius along the path (using determined start/end in meters)
        radius_z_interp = np.linspace(start_rz, end_rz, total_number_of_waypoints, endpoint=False)
        z_rel_mortar = self._calculate_ellipsoid_z(x_rel_mortar, y_rel_mortar, radius_z_interp)

        position_relative = np.stack([x_rel_mortar, y_rel_mortar, z_rel_mortar], axis=0) # Shape (3, N)

        # --- Calculate Global Positions ---
        mortar_center = self.mortar_top_center_position
        pos_global_x = position_relative[0] + mortar_center["x"]
        pos_global_y = position_relative[1] + mortar_center["y"]
        pos_global_z = position_relative[2] + mortar_center["z"]
        position_global = np.stack([pos_global_x, pos_global_y, pos_global_z], axis=0) # Shape (3, N)

        # --- Calculate Orientations ---
        total_yaw_twist = yaw_twist_per_rotation * number_of_rotations
        orientations = self._calculate_orientation(
            position_relative, radius_z_interp, angle_scale, yaw_bias, total_yaw_twist
        )

        # --- Create Final Waypoints ---
        waypoints = self._create_waypoints_array(position_global, orientations)

        return waypoints


    def create_cartesian_waypoints(
        self,
        beginning_position_mm: List[float], # [x, y] relative to mortar center (mm)
        end_position_mm: List[float],       # [x, y] relative to mortar center (mm)
        # Make z-radii optional, default to None
        beginning_radius_z_mm: Optional[float] = None, # Ellipsoid z-radius at start (mm). Defaults to mortar's inner z-radius if None.
        end_radius_z_mm: Optional[float] = None,       # Ellipsoid z-radius at end (mm). Defaults to mortar's inner z-radius if None.
        angle_scale: float = 0.5,
        fixed_quaternion: bool = False,
        yaw_bias: Optional[float] = None,
        number_of_waypoints: int = 10,
    ) -> np.ndarray:
        """
        Generates waypoints following a straight line in the XY plane projected onto the mortar surface.

        Args:
            beginning_position_mm: Start XY position relative to mortar center (mm).
            end_position_mm: End XY position relative to mortar center (mm).
            beginning_radius_z_mm: Ellipsoid Z-radius at the start (mm). Defaults to mortar's inner z-radius if None.
            end_radius_z_mm: Ellipsoid Z-radius at the end (mm). Defaults to mortar's inner z-radius if None.
            angle_scale: Tool orientation interpolation factor (0=vertical, 1=normal).
            fixed_quaternion: If True, use the orientation of the first waypoint for all points.
            yaw_bias: Fixed yaw offset (radians).
            number_of_waypoints: Number of points along the path.

        Returns:
            Array of waypoints (shape [N, 7]).

        Raises:
            ValueError: If input parameters are invalid or path goes outside mortar bounds.
        """
        if number_of_waypoints < 1:
             if number_of_waypoints == 0:
                 return np.empty((0, 7))
             # Allow 1 waypoint (start point only)
             # If 1, linspace needs endpoint=True

        # --- Set default Z radii if not provided ---
        # Get the default z-radius from the initialized mortar parameters (in meters)
        default_rz_m = self.mortar_inner_radii["z"]
        # Convert the default radius (meters) to millimeters for comparison and potential use
        default_rz_mm = default_rz_m / MM_TO_M

        # Use provided value or default if None
        start_rz_mm_to_use = beginning_radius_z_mm if beginning_radius_z_mm is not None else default_rz_mm
        end_rz_mm_to_use = end_radius_z_mm if end_radius_z_mm is not None else default_rz_mm

        # --- Check if end radius is smaller than beginning radius ---
        # This check should happen *after* defaults are applied
        if end_rz_mm_to_use < start_rz_mm_to_use:
             if not np.isclose(end_rz_mm_to_use, start_rz_mm_to_use):
                 raise ValueError(f"Effective end_radius_z_mm ({end_rz_mm_to_use:.2f}) cannot be smaller than effective beginning_radius_z_mm ({start_rz_mm_to_use:.2f}).")

        # --- Convert inputs to meters and numpy arrays ---
        start_pos_xy = np.array(beginning_position_mm) * MM_TO_M
        end_pos_xy = np.array(end_position_mm) * MM_TO_M
        # Convert the determined mm values (either provided or default) to meters
        start_rz = start_rz_mm_to_use * MM_TO_M
        end_rz = end_rz_mm_to_use * MM_TO_M

        # --- Calculate XY positions (relative to mortar center) ---
        endpoint = (number_of_waypoints == 1) # Only include endpoint if generating just 1 point
        x_rel_mortar, y_rel_mortar = lerp_in_cartesian(
            start_pos_xy, end_pos_xy, number_of_waypoints
        )
        if number_of_waypoints == 1:
             x_rel_mortar = np.array([start_pos_xy[0]])
             y_rel_mortar = np.array([start_pos_xy[1]])


        # --- Check if path exceeds mortar XY boundaries ---
        max_rx = self.mortar_inner_radii["x"]
        max_ry = self.mortar_inner_radii["y"]
        if np.any(np.abs(x_rel_mortar) > max_rx + 1e-6):
             exceeding_indices = np.where(np.abs(x_rel_mortar) > max_rx + 1e-6)[0]
             raise ValueError(f"Calculated X path exceeds mortar radius ({max_rx*1000:.1f} mm) at indices {exceeding_indices}.")
        if np.any(np.abs(y_rel_mortar) > max_ry + 1e-6):
             exceeding_indices = np.where(np.abs(y_rel_mortar) > max_ry + 1e-6)[0]
             raise ValueError(f"Calculated Y path exceeds mortar radius ({max_ry*1000:.1f} mm) at indices {exceeding_indices}.")

        # --- Calculate Z positions (relative to mortar center) ---
        # Use the determined start/end z-radii (in meters) for interpolation
        radius_z_interp = np.linspace(start_rz, end_rz, number_of_waypoints, endpoint=endpoint)
        z_rel_mortar = self._calculate_ellipsoid_z(x_rel_mortar, y_rel_mortar, radius_z_interp)

        position_relative = np.stack([x_rel_mortar, y_rel_mortar, z_rel_mortar], axis=0) # Shape (3, N)

        # --- Calculate Global Positions ---
        mortar_center = self.mortar_top_center_position
        pos_global_x = position_relative[0] + mortar_center["x"]
        pos_global_y = position_relative[1] + mortar_center["y"]
        pos_global_z = position_relative[2] + mortar_center["z"]
        position_global = np.stack([pos_global_x, pos_global_y, pos_global_z], axis=0) # Shape (3, N)

        # --- Calculate Orientations ---
        orientations = self._calculate_orientation(
            position_relative, radius_z_interp, angle_scale, yaw_bias, yaw_twist_total=0, fixed_quaternion=fixed_quaternion
        )

        # --- Create Final Waypoints ---
        waypoints = self._create_waypoints_array(position_global, orientations)

        return waypoints

    def create_linear_waypoints_list(
        self,
        beginning_theta: float, # Start angle (radians)
        end_theta: float,       # End angle (radians)
        beginning_length_from_center_mm: float, # Start radius (mm)
        end_length_from_center_mm: float,       # End radius (mm)
        # Make z-radii optional, default to None
        beginning_radius_z_mm: Optional[float] = None, # Ellipsoid z-radius at start radius (mm). Defaults to mortar's inner z-radius if None.
        end_radius_z_mm: Optional[float] = None,       # Ellipsoid z-radius at end radius (mm). Defaults to mortar's inner z-radius if None.
        angle_scale: float = 0.5,
        fixed_quaternion: bool = False,
        yaw_bias: Optional[float] = None,
        number_of_waypoints_per_line: int = 5,
        motion_counts: int = 1, # Number of lines
    ) -> List[np.ndarray]:
        """
        Generates a list of waypoint arrays, each representing a linear path segment
        defined in polar coordinates (constant theta, varying radius).

        Args:
            beginning_theta: Start angle for the range of lines (radians).
            end_theta: End angle for the range of lines (radians).
            beginning_length_from_center_mm: Start radius for each line (mm).
            end_length_from_center_mm: End radius for each line (mm).
            beginning_radius_z_mm: Ellipsoid Z-radius at the start radius of each line (mm). Defaults to mortar's inner z-radius if None.
            end_radius_z_mm: Ellipsoid Z-radius at the end radius of each line (mm). Defaults to mortar's inner z-radius if None.
            angle_scale: Tool orientation interpolation factor (0=vertical, 1=normal).
            fixed_quaternion: If True, use the orientation of the first waypoint for all points in a line.
            yaw_bias: Fixed yaw offset for all lines (radians).
            number_of_waypoints_per_line: Number of points along each line segment.
            motion_counts: Number of lines to generate between beginning_theta and end_theta.

        Returns:
            A list of waypoint arrays (each shape [N, 7]).

        Raises:
            ValueError: If input parameters are invalid.
        """
        if number_of_waypoints_per_line < 1:
            raise ValueError("number_of_waypoints_per_line must be >= 1")
        if motion_counts < 1:
            raise ValueError("motion_counts must be >= 1")

        # --- Set default Z radii if not provided ---
        default_rz_m = self.mortar_inner_radii["z"]
        default_rz_mm = default_rz_m / MM_TO_M
        start_rz_mm_to_use = beginning_radius_z_mm if beginning_radius_z_mm is not None else default_rz_mm
        end_rz_mm_to_use = end_radius_z_mm if end_radius_z_mm is not None else default_rz_mm

        # --- Check if end radius is smaller than beginning radius (after defaults) ---
        if end_rz_mm_to_use < start_rz_mm_to_use:
             if not np.isclose(end_rz_mm_to_use, start_rz_mm_to_use):
                 raise ValueError(f"Effective end_radius_z_mm ({end_rz_mm_to_use:.2f}) cannot be smaller than effective beginning_radius_z_mm ({start_rz_mm_to_use:.2f}).")

        # --- Convert scalar radial inputs to meters ---
        start_r_m = beginning_length_from_center_mm * MM_TO_M
        end_r_m = end_length_from_center_mm * MM_TO_M
        # Z-radii (start_rz_mm_to_use, end_rz_mm_to_use) remain in mm for the call below

        # --- Determine the angles for each line segment ---
        endpoint = (motion_counts > 1 or np.isclose(beginning_theta, end_theta))
        thetas = np.linspace(beginning_theta, end_theta, motion_counts, endpoint=endpoint)

        waypoints_list = []
        for theta in thetas:
            # --- Define start and end points in Cartesian (relative to mortar center) for the current line ---
            start_x_m, start_y_m = polar_to_cartesian(start_r_m, theta)
            end_x_m, end_y_m = polar_to_cartesian(end_r_m, theta)

            # --- Generate waypoints for this line using create_cartesian_waypoints ---
            # Convert start/end points back to mm for the function call
            # Pass the determined z-radii (mm) explicitly
            line_waypoints = self.create_cartesian_waypoints(
                beginning_position_mm=[start_x_m / MM_TO_M, start_y_m / MM_TO_M],
                end_position_mm=[end_x_m / MM_TO_M, end_y_m / MM_TO_M],
                beginning_radius_z_mm=start_rz_mm_to_use, # Pass determined mm value
                end_radius_z_mm=end_rz_mm_to_use,         # Pass determined mm value
                angle_scale=angle_scale,
                fixed_quaternion=fixed_quaternion,
                yaw_bias=yaw_bias, # Apply same yaw bias to all lines
                number_of_waypoints=number_of_waypoints_per_line,
            )
            waypoints_list.append(line_waypoints)

        return waypoints_list


# --- Main function for testing ---

def main():
    # Parameters
    # Mortar size (diameter/depth) in mm
    mortar_inner_size_mm = {
        "x": 80.0, # e.g., 80mm diameter
        "y": 80.0,
        "z": 70.0, # e.g., 70mm depth
    }
    # Mortar center position in meters
    mortar_top_center_pos_m = {
        "x": -0.2448,
        "y": 0.3722,
        "z": 0.0451,
    }

    # Create GrindingMotionGenerator instance
    try:
        motion_generator = GrindingMotionGenerator(
            mortar_top_center_pos_m, mortar_inner_size_mm
        )
        print("--- Mortar Parameters ---")
        print(f"Center Position (m): {motion_generator.mortar_top_center_position}")
        print(f"Inner Radii (m): x={motion_generator.mortar_inner_radii['x']:.4f}, "
              f"y={motion_generator.mortar_inner_radii['y']:.4f}, "
              f"z={motion_generator.mortar_inner_radii['z']:.4f}")
        print(f"Yaw Twist Limit (rad): [{motion_generator.min_abs_yaw_twist:.2f}, {motion_generator.max_abs_yaw_twist:.2f}]")
        center_m = np.array([motion_generator.mortar_top_center_position[k] for k in ['x','y','z']])
        default_rz_mm = motion_generator.mortar_inner_radii['z'] / MM_TO_M
        print(f"Default Z-Radius (mm): {default_rz_mm:.1f}")

    except ValueError as e:
        print(f"Error initializing generator: {e}")
        return

    # --- Test circular motion ---
    print("\n--- Testing circular motion (with default Z radii) ---")
    try:
        # Example: Spiral inwards on offset circle, using default Z radius
        circ_pos_begin_mm = [15.0, 0.0] # Start 15mm radius
        circ_pos_end_mm = [5.0, 0.0]   # End 5mm radius
        # beginning_radius_z_mm and end_radius_z_mm are omitted -> use default
        circ_center_offset_mm = [0.0, 0.0] # Centered circle

        waypoints_circ_def = motion_generator.create_circular_waypoints(
            beginning_position_mm=circ_pos_begin_mm,
            end_position_mm=circ_pos_end_mm,
            # beginning_radius_z_mm=None, # Explicitly None or omitted
            # end_radius_z_mm=None,       # Explicitly None or omitted
            angle_scale=0.7, # More towards normal
            yaw_bias=None,
            yaw_twist_per_rotation= -pi / 4, # -45 deg twist per rotation
            number_of_rotations=3,
            number_of_waypoints_per_circle=30,
            center_position_mm=circ_center_offset_mm
        )
        print(f"Circular waypoints (default Z) generated. Shape: {waypoints_circ_def.shape}")
        if waypoints_circ_def.shape[0] > 0:
            print(f"Example waypoints (first 3):\n{waypoints_circ_def[:3]}")
            rel_pos = waypoints_circ_def[:, :3] - center_m
            print(f"Position relative to mortar center (mm):")
            print(f"  X range: [{np.min(rel_pos[:, 0])*1000:.1f}, {np.max(rel_pos[:, 0])*1000:.1f}] (Radius: {motion_generator.mortar_inner_radii['x']*1000:.1f})")
            print(f"  Y range: [{np.min(rel_pos[:, 1])*1000:.1f}, {np.max(rel_pos[:, 1])*1000:.1f}] (Radius: {motion_generator.mortar_inner_radii['y']*1000:.1f})")
            print(f"  Z range: [{np.min(rel_pos[:, 2])*1000:.1f}, {np.max(rel_pos[:, 2])*1000:.1f}] (Default Radius: {default_rz_mm:.1f})")
        else:
            print("No waypoints generated.")

    except ValueError as e:
        print(f"ERROR generating circular waypoints (default Z): {e}")
    except Exception as e:
        print(f"UNEXPECTED ERROR during circular generation (default Z): {e}")
        import traceback
        traceback.print_exc()


    # --- Test cartesian motion ---
    print("\n--- Testing cartesian motion (with specified Z radii) ---")
    try:
        # Define a line across the mortar, e.g., from (-15, 0) to (15, 0) mm relative to center
        cart_pos_begin_mm = [-15.0, 0.0]
        cart_pos_end_mm = [15.0, 0.0]
        cart_rz_begin_mm = 35.0 # Specify Z-radius (mm)
        cart_rz_end_mm = 37.0   # Specify different end Z-radius (mm)

        waypoints_cart_spec = motion_generator.create_cartesian_waypoints(
            beginning_position_mm=cart_pos_begin_mm,
            end_position_mm=cart_pos_end_mm,
            angle_scale=0.1, # Mostly vertical
            fixed_quaternion=False,
            yaw_bias=pi/2, # 90 degree yaw
            number_of_waypoints=15,
        )
        print(f"Cartesian waypoints (specified Z) generated. Shape: {waypoints_cart_spec.shape}")
        if waypoints_cart_spec.shape[0] > 0:
            print(f"Example waypoints (first 3):\n{waypoints_cart_spec[:3]}")
            rel_pos_cart = waypoints_cart_spec[:, :3] - center_m
            print(f"Position relative to mortar center (mm):")
            print(f"  X range: [{np.min(rel_pos_cart[:, 0])*1000:.1f}, {np.max(rel_pos_cart[:, 0])*1000:.1f}]")
            print(f"  Y range: [{np.min(rel_pos_cart[:, 1])*1000:.1f}, {np.max(rel_pos_cart[:, 1])*1000:.1f}]")
            print(f"  Z range: [{np.min(rel_pos_cart[:, 2])*1000:.1f}, {np.max(rel_pos_cart[:, 2])*1000:.1f}] (Specified Z radii: {cart_rz_begin_mm:.1f} -> {cart_rz_end_mm:.1f})")
        else:
            print("No waypoints generated.")

    except ValueError as e:
        print(f"ERROR generating cartesian waypoints (specified Z): {e}")
    except Exception as e:
        print(f"UNEXPECTED ERROR during cartesian generation (specified Z): {e}")
        import traceback
        traceback.print_exc()

    print("\n--- Testing cartesian motion (with default Z radii) ---")
    try:
        # Same line, but omit Z radii to use default
        cart_pos_begin_mm = [-15.0, 0.0]
        cart_pos_end_mm = [15.0, 0.0]

        waypoints_cart_def = motion_generator.create_cartesian_waypoints(
            beginning_position_mm=cart_pos_begin_mm,
            end_position_mm=cart_pos_end_mm,
            # beginning_radius_z_mm=None, # Omitted
            # end_radius_z_mm=None,       # Omitted
            angle_scale=0.1,
            fixed_quaternion=False,
            yaw_bias=pi/2,
            number_of_waypoints=15,
        )
        print(f"Cartesian waypoints (default Z) generated. Shape: {waypoints_cart_def.shape}")
        if waypoints_cart_def.shape[0] > 0:
            print(f"Example waypoints (first 3):\n{waypoints_cart_def[:3]}")
            rel_pos_cart_def = waypoints_cart_def[:, :3] - center_m
            print(f"Position relative to mortar center (mm):")
            print(f"  X range: [{np.min(rel_pos_cart_def[:, 0])*1000:.1f}, {np.max(rel_pos_cart_def[:, 0])*1000:.1f}]")
            print(f"  Y range: [{np.min(rel_pos_cart_def[:, 1])*1000:.1f}, {np.max(rel_pos_cart_def[:, 1])*1000:.1f}]")
            print(f"  Z range: [{np.min(rel_pos_cart_def[:, 2])*1000:.1f}, {np.max(rel_pos_cart_def[:, 2])*1000:.1f}] (Default Radius: {default_rz_mm:.1f})")
        else:
            print("No waypoints generated.")

    except ValueError as e:
        print(f"ERROR generating cartesian waypoints (default Z): {e}")
    except Exception as e:
        print(f"UNEXPECTED ERROR during cartesian generation (default Z): {e}")
        import traceback
        traceback.print_exc()


    # --- Test linear (polar defined) motion list ---
    print("\n--- Testing linear (polar defined) motion list (with default Z radii) ---")
    try:
        waypoints_list_lin_def = motion_generator.create_linear_waypoints_list(
            beginning_theta=0.0,         # Start at 0 degrees
            end_theta=pi,              # Go up to 180 degrees
            beginning_length_from_center_mm=5.0,  # Start 5mm from center
            end_length_from_center_mm=20.0, # End 20mm from center
            # beginning_radius_z_mm=None, # Omitted
            # end_radius_z_mm=None,       # Omitted
            angle_scale=0.9,           # Mostly normal
            fixed_quaternion=True,     # Keep orientation constant along each line
            yaw_bias=0.0,              # No yaw bias
            number_of_waypoints_per_line=8,
            motion_counts=5,           # Generate 5 lines
        )
        print(f"Linear waypoints list (default Z) generated. Number of lists: {len(waypoints_list_lin_def)}")
        thetas = np.linspace(0.0, pi, 5, endpoint=True)
        for i, waypoints in enumerate(waypoints_list_lin_def):
            print(f"--- List {i} (theta ~ {thetas[i]:.2f} rad) ---")
            print(f"Shape: {waypoints.shape}")
            if waypoints.shape[0] > 0:
                print(f"Example waypoints (first 2):\n{waypoints[:2]}")
                rel_pos_lin = waypoints[:, :3] - center_m
                print(f"Position relative to mortar center (mm):")
                print(f"  X range: [{np.min(rel_pos_lin[:, 0])*1000:.1f}, {np.max(rel_pos_lin[:, 0])*1000:.1f}]")
                print(f"  Y range: [{np.min(rel_pos_lin[:, 1])*1000:.1f}, {np.max(rel_pos_lin[:, 1])*1000:.1f}]")
                print(f"  Z range: [{np.min(rel_pos_lin[:, 2])*1000:.1f}, {np.max(rel_pos_lin[:, 2])*1000:.1f}] (Default Radius: {default_rz_mm:.1f})")
            else:
                print("No waypoints generated for this list.")


    except ValueError as e:
        print(f"ERROR generating linear waypoints list (default Z): {e}")
    except Exception as e:
        print(f"UNEXPECTED ERROR during linear list generation (default Z): {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
