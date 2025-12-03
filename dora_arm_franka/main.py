import os
import time
import requests
import pyarrow as pa
from dora import Node

# --------------------------
# 1. 机械臂数据获取核心函数
# --------------------------
max_gripper_distance = 0.83
max_gripper_catch_distance = 0.7  # 夹持状态的最大距离（小于此值可能夹住物体）
min_gripper_distance = 0.02     # 夹持状态的最小距离（大于此值避免完全闭合）
GRASP_STABLE_DURATION = 0.8       # 稳定夹持的判断时间（秒）

def get_arm_data(url):
    """统一获取机械臂关节、夹爪、位姿数据，返回字典格式"""
    arm_data = {
        "jointstate": None,
        "gripper": None,  # 格式：[当前距离, 是否稳定夹持(0/1)]
        "pose": None,
        "success": False
    }

    try:
        # 获取关节角度
        joint_resp = requests.post(f"{url}getq", timeout=0.1)
        if joint_resp.status_code == 200:
            arm_data["jointstate"] = joint_resp.json()["q"]

        # 获取夹爪距离
        gripper_resp = requests.post(f"{url}get_gripper", timeout=0.1)
        if gripper_resp.status_code == 200:
            arm_data["gripper_raw"] = gripper_resp.json()["gripper"]  # 暂存原始距离

        # 获取末端位姿
        pose_resp = requests.post(f"{url}getpos_euler", timeout=0.1)
        if pose_resp.status_code == 200:
            pose = pose_resp.json()["pose"]
            arm_data["pose"] = [pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]]

        # 标记数据是否完整
        if all(v is not None for v in [arm_data["jointstate"], arm_data.get("gripper_raw"), arm_data["pose"]]):
            arm_data["success"] = True

    except requests.exceptions.RequestException as e:
        print(f"机械臂API请求失败: {e}")

    return arm_data

# --------------------------
# 2. 主逻辑（事件循环+状态跟踪）
# --------------------------
def main():
    arm_url = os.getenv("url", "http://127.0.0.1:5000/")
    print(f"机械臂API地址: {arm_url}")

    node = Node()

    # 状态跟踪变量（关键改进）
    grasp_state = {
        "in_range": False,          # 当前是否在目标距离范围内
        "enter_time": None,         # 进入范围的时间戳（首次进入时记录）
        "is_stable_grasp": False    # 是否达到稳定夹持状态（持续1秒以上）
    }

    # 事件循环
    for event in node:
        if event["type"] == "INPUT" and event["id"] == "tick":
            arm_data = get_arm_data(arm_url)
            
            if arm_data["success"]:
                cur_gripper = arm_data["gripper_raw"]
                current_time = time.time()  # 当前时间戳（秒）

                # 1. 判断当前是否在目标范围内
                cur_in_range = (min_gripper_distance < cur_gripper < max_gripper_catch_distance)

                # 2. 更新状态跟踪（核心逻辑）
                if cur_in_range:
                    # 刚进入范围：记录进入时间
                    if not grasp_state["in_range"]:
                        grasp_state["enter_time"] = current_time
                        grasp_state["in_range"] = True
                        grasp_state["is_stable_grasp"] = False  # 刚进入，尚未稳定
                    # 已在范围内：检查是否稳定1秒以上
                    else:
                        duration = current_time - grasp_state["enter_time"]
                        if duration >= GRASP_STABLE_DURATION:
                            grasp_state["is_stable_grasp"] = True  # 稳定夹持确认
                else:
                    # 不在范围内：重置状态
                    grasp_state["in_range"] = False
                    grasp_state["enter_time"] = None
                    grasp_state["is_stable_grasp"] = False

                # 3. 组装夹爪数据（距离+稳定夹持标志）
                arm_data["gripper"] = [cur_gripper, 1 if grasp_state["is_stable_grasp"] else 0]

                # 4. 拼接所有数据并发送
                combined_list = (
                    arm_data["jointstate"]
                    + arm_data["gripper"]
                    + arm_data["pose"]
                )
                # print(f"夹爪状态: 距离={cur_gripper:.4f}, 稳定夹持={grasp_state['is_stable_grasp']}")
                node.send_output(
                    "jointstate",
                    pa.array(combined_list, type=pa.float32()),
                    {"timestamp": time.time_ns()}
                )

        elif event["type"] == "INPUT" and event["id"] == "stop":
            print("收到停止指令，停止机械臂...")

    print("Dora节点退出，清理资源...")

if __name__ == "__main__":
    main()