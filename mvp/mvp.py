"""
MVP Test Script for RealMan Robotic Arm
Tests: moves, movej_canfd, and lift functions with small movements for safety.
Uses threading for parallel dual-arm control.
"""

import time
import threading
from Robotic_Arm.rm_robot_interface import *


def connect_robot(ip, port, level=3, mode=2):
    """
    Connect to the robot arm.

    Args:
        ip (str): IP address of the robot arm.
        port (int): Port number.
        level (int, optional): Connection level. Defaults to 3.
        mode (int, optional): Thread mode (0: single, 1: dual, 2: triple). Defaults to 2.

    Returns:
        RoboticArm: Instance of the connected RoboticArm, or None if failed.
    """
    thread_mode = rm_thread_mode_e(mode)
    robot = RoboticArm(thread_mode)
    handle = robot.rm_create_robot_arm(ip, port, level)

    if handle.id == -1:
        print(f"\nFailed to connect to robot at {ip}:{port}\n")
        return None
    else:
        print(f"\nSuccessfully connected to robot at {ip}:{port}, handle ID: {handle.id}\n")
        return robot


def disconnect_robot(robot):
    """Disconnect from the robot arm."""
    if robot is None:
        return
    result = robot.rm_delete_robot_arm()
    if result == 0:
        print("\nSuccessfully disconnected from the robot arm\n")
    else:
        print("\nFailed to disconnect from the robot arm\n")


def get_arm_info(robot):
    """Get robot arm info including model and DOF."""
    res, info = robot.rm_get_robot_info()
    if res == 0:
        return info
    else:
        print(f"Failed to get robot info, error code: {res}")
        return None


def get_current_state(robot):
    """Get current arm state (joints and pose)."""
    res, state = robot.rm_get_current_arm_state()
    if res == 0:
        return state
    else:
        print(f"Failed to get current state, error code: {res}")
        return None


def test_movej(robot, arm_name="Robot"):
    """
    Test movej function - joint space motion.
    Moves joints by small angles.

    Args:
        robot (RoboticArm): Robot instance.
        arm_name (str): Name for logging.
    """
    print(f"\n[{arm_name}] ========== Testing movej (Joint Space Motion) ==========")

    state = get_current_state(robot)
    if state is None:
        print(f"[{arm_name}] Failed to get current state, skipping movej test")
        return

    current_joints = state['joint']

    arm_info = get_arm_info(robot)
    if arm_info is None:
        print(f"[{arm_name}] Failed to get arm info, skipping movej test")
        return

    dof = arm_info['arm_dof']
    print(f"[{arm_name}] Current joints: {[round(j, 2) for j in current_joints[:dof]]} deg")

    # Small movement: move joint 2 by +5 degrees, then back
    joint_offset = 5.0  # 5 degrees
    speed = 10  # 10% speed percentage

    # Target position: joint 2 + 5 degrees
    target_joints = list(current_joints[:dof])
    target_joints[1] = current_joints[1] + joint_offset  # joint 2 (index 1) + 5 deg

    print(f"[{arm_name}] Moving joint 2 by +{joint_offset} deg...")
    result = robot.rm_movej(target_joints, speed, 0, 0, 1)  # speed: 10%, joints: degrees
    if result != 0:
        print(f"[{arm_name}] movej failed, error: {result}")
        return
    print(f"[{arm_name}] movej to target succeeded")

    time.sleep(0.5)  # 500ms delay

    # Return to original position
    print(f"[{arm_name}] Returning to original joint positions...")
    result = robot.rm_movej(list(current_joints[:dof]), speed, 0, 0, 1)  # speed: 10%, joints: degrees
    if result != 0:
        print(f"[{arm_name}] movej return failed, error: {result}")
        return

    print(f"[{arm_name}] movej test completed successfully")


def test_movej_p(robot, arm_name="Robot"):
    """
    Test movej_p function - joint space motion to target pose.
    Moves to a pose offset by small distance.

    Args:
        robot (RoboticArm): Robot instance.
        arm_name (str): Name for logging.
    """
    print(f"\n[{arm_name}] ========== Testing movej_p (Joint Motion to Pose) ==========")

    state = get_current_state(robot)
    if state is None:
        print(f"[{arm_name}] Failed to get current state, skipping movej_p test")
        return

    current_pose = state['pose']
    print(f"[{arm_name}] Current pose: x={current_pose[0]:.3f}m, y={current_pose[1]:.3f}m, z={current_pose[2]:.3f}m")

    # Small movement: move Z axis by +2cm
    z_offset = 0.02  # 2 cm = 0.02 meters
    speed = 10  # 10% speed percentage

    # Target pose: Z + 2cm
    target_pose = list(current_pose)
    target_pose[2] = current_pose[2] + z_offset  # Z + 0.02m

    print(f"[{arm_name}] Moving Z by +{z_offset * 100}cm to z={target_pose[2]:.3f}m...")
    result = robot.rm_movej_p(target_pose, speed, 0, 0, 1)  # speed: 10%, position: meters, orientation: radians
    if result != 0:
        print(f"[{arm_name}] movej_p failed, error: {result}")
        return
    print(f"[{arm_name}] movej_p to target succeeded")

    time.sleep(0.5)  # 500ms delay

    # Return to original position
    print(f"[{arm_name}] Returning to original pose...")
    result = robot.rm_movej_p(list(current_pose), speed, 0, 0, 1)  # speed: 10%, position: meters, orientation: radians
    if result != 0:
        print(f"[{arm_name}] movej_p return failed, error: {result}")
        return

    print(f"[{arm_name}] movej_p test completed successfully")


def test_moves(robot, arm_name="Robot"):
    """
    Test moves function with small Cartesian space movements.
    Moves in a small triangle pattern.

    Args:
        robot (RoboticArm): Robot instance.
        arm_name (str): Name for logging.
    """
    print(f"\n[{arm_name}] ========== Testing moves (Spline Curve Motion) ==========")

    state = get_current_state(robot)
    if state is None:
        print(f"[{arm_name}] Failed to get current state, skipping moves test")
        return

    current_pose = state['pose']
    print(f"[{arm_name}] Current pose: x={current_pose[0]:.3f}m, y={current_pose[1]:.3f}m, z={current_pose[2]:.3f}m")

    # Small movement offsets: 2cm = 0.02m
    offset = 0.02  # 2 cm = 0.02 meters

    # Create 3 waypoints forming a small triangle (for spline, need at least 3 points)
    waypoints = [
        [current_pose[0] + offset, current_pose[1], current_pose[2],
         current_pose[3], current_pose[4], current_pose[5]],  # +2cm in X direction
        [current_pose[0] + offset, current_pose[1] + offset, current_pose[2],
         current_pose[3], current_pose[4], current_pose[5]],  # +2cm in Y direction
        [current_pose[0], current_pose[1], current_pose[2],
         current_pose[3], current_pose[4], current_pose[5]],  # back to start
    ]

    speed = 10  # 10% speed percentage

    # Execute spline motion (connect=1 for all except last point)
    for i, wp in enumerate(waypoints):
        connect = 1 if i < len(waypoints) - 1 else 0  # connect trajectory except last
        result = robot.rm_moves(wp, speed, 0, connect, 1)  # speed: 10%, position: meters, orientation: radians
        if result == 0:
            print(f"[{arm_name}] moves waypoint {i} succeeded, x={wp[0]:.3f}m, y={wp[1]:.3f}m, z={wp[2]:.3f}m")
        else:
            print(f"[{arm_name}] moves failed at waypoint {i}, error: {result}")
            return
        time.sleep(0.1)  # 100ms delay between commands

    print(f"[{arm_name}] moves test completed successfully")


def test_movej_canfd(robot, arm_name="Robot"):
    """
    Test movej_canfd function with small joint movements.
    Performs a small oscillation on joint 1.

    Args:
        robot (RoboticArm): Robot instance.
        arm_name (str): Name for logging.
    """
    print(f"\n[{arm_name}] ========== Testing movej_canfd (CAN FD Pass-through) ==========")

    state = get_current_state(robot)
    if state is None:
        print(f"[{arm_name}] Failed to get current state, skipping movej_canfd test")
        return

    current_joints = state['joint']

    arm_info = get_arm_info(robot)
    if arm_info is None:
        print(f"[{arm_name}] Failed to get arm info, skipping movej_canfd test")
        return

    dof = arm_info['arm_dof']
    print(f"[{arm_name}] Current joints: {[round(j, 2) for j in current_joints[:dof]]} deg")
    print(f"[{arm_name}] Arm DOF: {dof}")

    # Small movement: oscillate joint 1 by +/- 2 degrees
    oscillation = 2.0  # 2 degrees
    num_steps = 20  # number of interpolation steps

    # Create smooth trajectory: current -> +2deg -> current
    trajectory = []

    # Phase 1: current to +2 degrees on joint 1
    for i in range(num_steps):
        ratio = i / num_steps
        joints = list(current_joints[:dof])
        joints[0] = current_joints[0] + oscillation * ratio  # joint angle in degrees
        trajectory.append(joints)

    # Phase 2: +2 degrees back to current
    for i in range(num_steps):
        ratio = i / num_steps
        joints = list(current_joints[:dof])
        joints[0] = current_joints[0] + oscillation * (1 - ratio)  # joint angle in degrees
        trajectory.append(joints)

    print(f"[{arm_name}] Executing {len(trajectory)} points via CANFD (low-follow mode)")
    print(f"[{arm_name}] Joint 1 will move +{oscillation} deg then back")

    # Execute pass-through
    for i, joints in enumerate(trajectory):
        result = robot.rm_movej_canfd(joints, follow=False)  # low-follow mode, joints in degrees
        if result != 0:
            print(f"[{arm_name}] movej_canfd failed at step {i}, error: {result}")
            return
        time.sleep(0.02)  # 20ms period (50Hz), required for low-follow mode

    print(f"[{arm_name}] movej_canfd test completed successfully")


def test_lift(robot, arm_name="Robot"):
    """
    Test lift function with small height changes.

    Args:
        robot (RoboticArm): Robot instance.
        arm_name (str): Name for logging.
    """
    print(f"\n[{arm_name}] ========== Testing Lift (Height Control) ==========")

    # Get current lift state
    result = robot.rm_get_lift_state()
    if result[0] != 0:
        print(f"[{arm_name}] Lift may not be available on this robot, skipping lift test")
        return

    lift_state = result[1]
    current_height = lift_state.get('height', 0)
    print(f"[{arm_name}] Current lift height: {current_height}mm")

    # Small movement: +/- 50mm from current position
    movement = 50  # 50 millimeters
    speed = 10  # 10% speed percentage

    # Calculate safe target heights (within 0-2600mm range)
    target_up = min(current_height + movement, 2600)  # height in mm, max 2600mm
    target_down = max(current_height - movement, 0)   # height in mm, min 0mm

    print(f"[{arm_name}] Moving lift up to {target_up}mm...")
    result = robot.rm_set_lift_height(speed, target_up, 1)  # speed: 10%, height: mm
    if result != 0:
        print(f"[{arm_name}] Lift up failed, error: {result}")
        return
    time.sleep(0.5)  # 500ms delay

    print(f"[{arm_name}] Moving lift down to {target_down}mm...")
    result = robot.rm_set_lift_height(speed, target_down, 1)  # speed: 10%, height: mm
    if result != 0:
        print(f"[{arm_name}] Lift down failed, error: {result}")
        return
    time.sleep(0.5)  # 500ms delay

    print(f"[{arm_name}] Returning lift to original height {current_height}mm...")
    result = robot.rm_set_lift_height(speed, current_height, 1)  # speed: 10%, height: mm
    if result != 0:
        print(f"[{arm_name}] Lift return failed, error: {result}")
        return

    print(f"[{arm_name}] Lift test completed successfully")


def run_arm_tests(ip, port, arm_name):
    """
    Run all tests on a single robot arm.

    Args:
        ip (str): Robot IP address.
        port (int): Robot port.
        arm_name (str): Name for logging (e.g., "LEFT", "RIGHT").
    """
    print(f"\n>>> [{arm_name}] Connecting to {ip}:{port}...")
    robot = connect_robot(ip, port, level=3, mode=2)

    if robot is None:
        print(f"[{arm_name}] Connection failed, skipping tests")
        return

    # Get arm info
    info = get_arm_info(robot)
    if info:
        print(f"[{arm_name}] Arm Model: {info['arm_model']}")
        print(f"[{arm_name}] Arm DOF: {info['arm_dof']}")

    try:
        test_movej(robot, arm_name)
        time.sleep(1)  # 1 second delay between tests

        test_movej_p(robot, arm_name)
        time.sleep(1)  # 1 second delay between tests

        test_moves(robot, arm_name)
        time.sleep(1)  # 1 second delay between tests

        test_movej_canfd(robot, arm_name)
        time.sleep(1)  # 1 second delay between tests

        test_lift(robot, arm_name)
    except Exception as e:
        print(f"[{arm_name}] Test error: {e}")

    disconnect_robot(robot)
    print(f"[{arm_name}] All tests completed")


def main():
    # ==================== Connection Configuration ====================
    # Dual arm setup - modify IP/port as needed
    LEFT_ARM_IP = "192.168.1.33"
    LEFT_ARM_PORT = 8082
    RIGHT_ARM_IP = "192.168.1.33"
    RIGHT_ARM_PORT = 8083

    print("=" * 60)
    print("RealMan AIDA Robot Arm MVP Test, Upper body.")
    print("Testing: movej, movej_p, moves, movej_canfd, lift")
    print("Using parallel threading for dual-arm control")
    print("=" * 60)

    print("\nAPI Version:", rm_api_version())

    # Create threads for parallel dual-arm control
    thread_left = threading.Thread(target=run_arm_tests, args=(LEFT_ARM_IP, LEFT_ARM_PORT, "LEFT"))
    thread_right = threading.Thread(target=run_arm_tests, args=(RIGHT_ARM_IP, RIGHT_ARM_PORT, "RIGHT"))

    # Start both threads (arms move in parallel)
    thread_left.start()
    thread_right.start()

    # Wait for both threads to complete
    thread_left.join()
    thread_right.join()

    print("\n" + "=" * 60)
    print("Both robot arm tests completed")
    print("=" * 60)


if __name__ == "__main__":
    main()
