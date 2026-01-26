from Robotic_Arm.rm_robot_interface import *

# 连接左臂
robot_left = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle_left = robot_left.rm_create_robot_arm("192.168.1.33", 8082)
print("Left arm ID：", handle_left.id)

# 连接右臂
robot_right = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle_right = robot_right.rm_create_robot_arm("192.168.1.33", 8083)
print("Right arm ID：", handle_right.id)

# 获取左臂软件信息
software_info_left = robot_left.rm_get_arm_software_info()
if software_info_left[0] == 0:
    print("\n================== Left arm Software Information ==================")
    print("Arm Model: ", software_info_left[1]['product_version'])
    print("Algorithm Library Version: ", software_info_left[1]['algorithm_info']['version'])
    print("Control Layer Software Version: ", software_info_left[1]['ctrl_info']['version'])
    print("Dynamics Version: ", software_info_left[1]['dynamic_info']['model_version'])
    print("Planning Layer Software Version: ", software_info_left[1]['plan_info']['version'])
    print("==============================================================\n")
else:
    print("\nFailed to get left arm software information, Error code: ", software_info_left[0], "\n")

# 获取右臂软件信息
software_info_right = robot_right.rm_get_arm_software_info()
if software_info_right[0] == 0:
    print("\n================== Right arm Software Information ==================")
    print("Arm Model: ", software_info_right[1]['product_version'])
    print("Algorithm Library Version: ", software_info_right[1]['algorithm_info']['version'])
    print("Control Layer Software Version: ", software_info_right[1]['ctrl_info']['version'])
    print("Dynamics Version: ", software_info_right[1]['dynamic_info']['model_version'])
    print("Planning Layer Software Version: ", software_info_right[1]['plan_info']['version'])
    print("==============================================================\n")
else:
    print("\nFailed to get right arm software information, Error code: ", software_info_right[0], "\n")

# 断开连接
robot_left.rm_delete_robot_arm()
robot_right.rm_delete_robot_arm()
