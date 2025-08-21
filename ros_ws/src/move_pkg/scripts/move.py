#!/usr/bin/env python3

import rospy
import json
import math
import os
from move_pkg.srv import planArmTrajectoryCubicSpline, planArmTrajectoryCubicSplineRequest
from move_pkg.srv import changeArmCtrlMode, changeArmCtrlModeRequest
from move_pkg.srv import ExecuteAction, ExecuteActionResponse
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from move_pkg.msg import sensorsData

# 全局变量
current_arm_joint_state = []
joint_state = JointState()
last_publish_time = None
trajectory_end_time = 0.0
is_executing = False  # 动作执行状态标志（避免并发冲突）
FIXED_ACTION_DIR = "/home/lab/actions"  # 固定动作文件路径

# 控制模式常量
ARM_CTRL_MODE_KEEP_POSE = 0
ARM_CTRL_MODE_AUTO_SWING = 1
ARM_CTRL_MODE_EXTERNAL = 2

# 手臂关节配置
ARM_JOINT_NAMES = [
    "l_arm_pitch", "l_arm_roll", "l_arm_yaw", "l_forearm_pitch",
    "l_hand_yaw", "l_hand_pitch", "l_hand_roll",
    "r_arm_pitch", "r_arm_roll", "r_arm_yaw", "r_forearm_pitch",
    "r_hand_yaw", "r_hand_pitch", "r_hand_roll"
]
ARM_JOINT_INDEX_RANGE = slice(0, 14)


def deg_to_rad(deg):
    return math.radians(deg)


def sensors_data_callback(msg):
    global current_arm_joint_state
    current_arm_joint_state = msg.joint_data.joint_q[12:26]
    current_arm_joint_state = [round(pos, 2) for pos in current_arm_joint_state]


def traj_callback(msg):
    global joint_state, last_publish_time
    if len(msg.points) == 0:
        return
    point = msg.points[-1] if len(msg.points) > 1 else msg.points[0]
    joint_state.name = ARM_JOINT_NAMES
    joint_state.position = [math.degrees(pos) for pos in point.positions[:14]]
    joint_state.velocity = [math.degrees(vel) for vel in point.velocities[:14]]
    joint_state.effort = [0] * 14
    last_publish_time = rospy.get_time()


def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=5.0)
        change_arm_ctrl_mode = rospy.ServiceProxy(service_name, changeArmCtrlMode)
        response = change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        
        if response.result:
            mode_names = {
                ARM_CTRL_MODE_KEEP_POSE: "保持姿势",
                ARM_CTRL_MODE_AUTO_SWING: "自动摆臂",
                ARM_CTRL_MODE_EXTERNAL: "外部控制"
            }
            rospy.loginfo("成功切换手臂控制模式为: %d (%s)", arm_ctrl_mode, mode_names.get(arm_ctrl_mode, "未知模式"))
            return True
        else:
            rospy.logerr("切换控制模式失败：%s", response.message)
            return False
            
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("切换控制模式失败: %s", str(e))
        return False


def load_tact_file(action_name, speed_scale=10.0):  # 核心修改：默认缩放10倍（速度变为1/10）
    """加载固定路径下的动作文件（仅需传入文件名）"""
    tact_file = os.path.join(FIXED_ACTION_DIR, f"{action_name}.tact")
    if not os.path.exists(tact_file):
        rospy.logerr("动作文件不存在：%s", tact_file)
        return [], [], 0.0
    
    try:
        with open(tact_file, 'r') as f:
            tact_data = json.load(f)
        
        if "frames" not in tact_data:
            rospy.logerr("Tact文件缺少'frames'字段")
            return [], [], 0.0
        
        frames = tact_data["frames"]
        unique_frames = []
        seen_servos = set()
        for frame in frames:
            if "servos" not in frame or "keyframe" not in frame:
                rospy.logwarn("跳过无效帧（缺少servos或keyframe）")
                continue
            servo_tuple = tuple(frame["servos"][ARM_JOINT_INDEX_RANGE])
            if servo_tuple not in seen_servos:
                seen_servos.add(servo_tuple)
                unique_frames.append(frame)
        
        positions = []
        times = []
        for frame in unique_frames:
            arm_servos = frame["servos"][ARM_JOINT_INDEX_RANGE]
            if len(arm_servos) != 14:
                rospy.logwarn("关节数量不符（预期14个，实际%d个）", len(arm_servos))
                continue
            arm_rad = [deg_to_rad(angle) for angle in arm_servos]
            positions.append(arm_rad)
            # 核心修改：时间乘以缩放因子（10倍），使速度变为原来的1/10
            time_sec = (frame["keyframe"] / 1000.0) * speed_scale
            times.append(time_sec)
        
        if not positions or not times:
            rospy.logerr("未提取到有效轨迹数据")
            return [], [], 0.0
        
        sorted_pairs = sorted(zip(times, positions), key=lambda x: x[0])
        times, positions = zip(*sorted_pairs)
        times = list(times)
        positions = list(positions)
        
        # 核心修改：总时长同样乘以缩放因子
        total_time = (tact_data.get("finish", 0) / 1000.0) * speed_scale if "finish" in tact_data else times[-1]
        if total_time < times[-1]:
            rospy.logwarn("finish字段时间小于最后一帧时间，使用最后一帧时间作为总时长")
            total_time = times[-1]
        
        rospy.loginfo("成功加载轨迹：%d个关键帧，总时长%.2f秒（速度为原始1/10）", len(positions), total_time)
        return positions, times, total_time
    
    except json.JSONDecodeError:
        rospy.logerr("Tact文件格式错误（非有效JSON）")
        return [], [], 0.0
    except Exception as e:
        rospy.logerr("解析Tact文件失败：%s", str(e))
        return [], [], 0.0


def plan_arm_traj_cubicspline(positions, times):
    if not positions or not times or len(positions) != len(times):
        rospy.logerr("轨迹点或时间无效")
        return False
    
    try:
        rospy.wait_for_service('/cubic_spline/plan_arm_trajectory', timeout=10.0)
        plan_service = rospy.ServiceProxy(
            '/cubic_spline/plan_arm_trajectory', 
            planArmTrajectoryCubicSpline
        )
    except rospy.ROSException:
        rospy.logerr("轨迹规划服务不可用")
        return False
    
    request = planArmTrajectoryCubicSplineRequest()
    joint_trajectory = JointTrajectory()
    joint_trajectory.joint_names = ARM_JOINT_NAMES
    
    for i in range(len(times)):
        point = JointTrajectoryPoint()
        point.positions = positions[i]
        point.time_from_start = rospy.Duration(times[i])
        point.velocities = [0.5] * 14
        joint_trajectory.points.append(point)
    
    request.joint_trajectory = joint_trajectory
    
    try:
        response = plan_service(request)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr("轨迹规划失败：%s", str(e))
        return False


def execute_action_callback(req):
    """服务回调函数：接收动作名称并执行"""
    global is_executing, trajectory_end_time
    if is_executing:
        return ExecuteActionResponse(success=False, message="正在执行其他动作，请稍后再试")
    
    is_executing = True  # 标记为正在执行
    action_name = req.action_name
    rospy.loginfo(f"收到动作请求：{action_name}")
    
    # 初始化发布者和订阅者（仅在首次执行时初始化）
    arm_traj_pub = rospy.Publisher(
        '/kuavo_arm_traj', 
        JointState, 
        queue_size=5, 
        tcp_nodelay=True
    )
    
    try:
        # 切换到外部控制模式
        if not call_change_arm_ctrl_mode_service(ARM_CTRL_MODE_EXTERNAL):
            is_executing = False
            return ExecuteActionResponse(success=False, message="无法切换到外部控制模式")
        
        # 等待获取当前关节状态
        rospy.loginfo("等待当前关节状态...")
        start_wait = rospy.get_time()
        while len(current_arm_joint_state) == 0 and (rospy.get_time() - start_wait) < 5.0:
            rospy.sleep(0.1)
        if len(current_arm_joint_state) == 0:
            if call_change_arm_ctrl_mode_service(ARM_CTRL_MODE_KEEP_POSE):
                rospy.loginfo("已切换到保持姿势模式，再次尝试获取关节状态")
                start_wait = rospy.get_time()
                while len(current_arm_joint_state) == 0 and (rospy.get_time() - start_wait) < 5.0:
                    rospy.sleep(0.1)
            if len(current_arm_joint_state) == 0:
                is_executing = False
                return ExecuteActionResponse(success=False, message="超时未获取到当前关节状态")
        
        # 加载动作文件（使用默认的10倍缩放，速度为原来1/10）
        positions, times, total_time = load_tact_file(action_name)
        if not positions or not times:
            is_executing = False
            return ExecuteActionResponse(success=False, message="动作文件解析失败")
        
        # 计算轨迹时间（保持原有逻辑不变）
        start_delay = 1.0
        trajectory_end_time = total_time + start_delay
        adjusted_times = [t + start_delay for t in times]
        adjusted_times.insert(0, 0.0)
        positions.insert(0, current_arm_joint_state)
        
        # 执行轨迹规划
        rospy.loginfo(f"规划轨迹（{len(positions)}个关键点，总时长{trajectory_end_time:.2f}秒）...")
        if not plan_arm_traj_cubicspline(positions, adjusted_times):
            rospy.logwarn("初次规划失败，尝试简化轨迹...")
            simplified_positions = []
            simplified_times = []
            prev_pos = None
            for pos, time in zip(positions, adjusted_times):
                if prev_pos is None or pos != prev_pos:
                    simplified_positions.append(pos)
                    simplified_times.append(time)
                    prev_pos = pos
            if len(simplified_positions) < 2:
                is_executing = False
                return ExecuteActionResponse(success=False, message="简化后轨迹点不足，无法规划")
            rospy.loginfo(f"简化后轨迹：{len(simplified_positions)}个关键点")
            if not plan_arm_traj_cubicspline(simplified_positions, simplified_times):
                is_executing = False
                return ExecuteActionResponse(success=False, message="简化后轨迹规划仍失败")
        
        # 执行轨迹
        rate = rospy.Rate(100)
        start_time = rospy.get_time()
        rospy.loginfo("开始执行动作...")
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time()
            elapsed = current_time - start_time
            
            # 退出条件1：动作执行完毕
            if elapsed > trajectory_end_time:
                rospy.loginfo(f"动作执行完毕（总时长{elapsed:.2f}秒）")
                break
            
            # 退出条件2：超时无新数据（保持原有逻辑）
            if last_publish_time is not None and (current_time - last_publish_time) > 15.0:
                rospy.logwarn("动作超时无更新，强制退出")
                is_executing = False
                return ExecuteActionResponse(success=False, message="动作超时无更新")
            
            # 发布关节状态
            try:
                if len(joint_state.position) == 14:
                    arm_traj_pub.publish(joint_state)
            except Exception as e:
                rospy.logerr(f"发布关节状态失败：{str(e)}")
            
            rate.sleep()
        
        # 动作结束后恢复为保持姿势模式
        if not call_change_arm_ctrl_mode_service(ARM_CTRL_MODE_KEEP_POSE):
            rospy.logwarn("无法切换到保持姿势模式")
        
        is_executing = False
        return ExecuteActionResponse(success=True, message=f"动作{action_name}执行成功")
    
    except Exception as e:
        is_executing = False
        rospy.logerr(f"动作执行异常：{str(e)}")
        return ExecuteActionResponse(success=False, message=f"执行异常：{str(e)}")


def main():
    # 初始化节点（持续运行）
    rospy.init_node('arm_trajectory_tact_executor', anonymous=False)
    
    # 初始化订阅者（持续监听传感器数据和轨迹）
    rospy.Subscriber(
        '/cubic_spline/arm_traj', 
        JointTrajectory, 
        traj_callback, 
        queue_size=5, 
        tcp_nodelay=True
    )
    rospy.Subscriber(
        '/sensors_data_raw', 
        sensorsData, 
        sensors_data_callback, 
        queue_size=1, 
        tcp_nodelay=True
    )
    
    # 注册服务：接收动作请求
    rospy.Service('execute_arm_action', ExecuteAction, execute_action_callback)
    rospy.loginfo("动作执行服务已启动，等待请求...（固定路径：/home/lab/actions）")
    
    # 持续运行
    rospy.spin()


if __name__ == "__main__":
    main()
