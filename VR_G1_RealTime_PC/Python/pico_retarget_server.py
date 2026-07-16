#!/usr/bin/env python3
"""
Real-time PICO → G1 Retargeting Server

从Unity stdin接收24点JSON数据，使用GeneralMotionRetargeting进行retarget，
输出G1 qpos JSON到Unity stdout。

使用方法:
    python pico_retarget_server.py

Unity通过Process.Start()调用此脚本，通过stdin/stdout通信。
"""

import sys
import json
import numpy as np
from scipy.spatial.transform import Rotation as R

# 添加GMR路径（根据您的实际路径调整）
GMR_PATH = r"C:\Users\Wayne\Desktop\GMU\PhD\GMR"
sys.path.insert(0, GMR_PATH)

from general_motion_retargeting import GeneralMotionRetargeting

# PICO 24点关节名称（和pico_csv_to_robot.py一致）
BODY_JOINT_NAMES = [
    "Pelvis",
    "Left_Hip",
    "Right_Hip",
    "Spine1",
    "Left_Knee",
    "Right_Knee",
    "Spine2",
    "Left_Ankle",
    "Right_Ankle",
    "Spine3",
    "Left_Foot",
    "Right_Foot",
    "Neck",
    "Left_Collar",
    "Right_Collar",
    "Head",
    "Left_Shoulder",
    "Right_Shoulder",
    "Left_Elbow",
    "Right_Elbow",
    "Left_Wrist",
    "Right_Wrist",
    "Left_Hand",
    "Right_Hand",
]


def unity_quat_to_wxyz(xyzw):
    """Convert Unity quaternion (xyzw) to scipy/MuJoCo (wxyz)"""
    return [xyzw[3], xyzw[0], xyzw[1], xyzw[2]]


def coordinate_transform_unity_data(body_pose_dict):
    """
    Transform from Unity (left-hand) to right-hand (wxyz quats).
    Same as XRobotStreamer.coordinate_transform_unity_data.
    """
    rotation_matrix = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    rotation_quat_scipy = R.from_matrix(rotation_matrix)

    for body_name, value in body_pose_dict.items():
        x, y, z = value[0]
        qw, qx, qy, qz = value[1]  # wxyz format

        # Rotate orientation
        quat_scipy = R.from_quat([qx, qy, qz, qw])  # scipy uses xyzw
        rotated_quat_scipy = rotation_quat_scipy * quat_scipy
        rotated_quat_xyzw = rotated_quat_scipy.as_quat()  # xyzw format
        rotated_quat_wxyz = [rotated_quat_xyzw[3], rotated_quat_xyzw[0], rotated_quat_xyzw[1], rotated_quat_xyzw[2]]  # convert to wxyz

        # Rotate position
        position = np.array([x, y, z]) @ rotation_matrix.T

        body_pose_dict[body_name][0] = position.tolist()
        body_pose_dict[body_name][1] = rotated_quat_wxyz  # Already a list

    return body_pose_dict


def main():
    # === 初始化GMR (只需一次) ===
    print("[RetargetServer] Initializing GeneralMotionRetargeting...", file=sys.stderr, flush=True)

    try:
        gmr = GeneralMotionRetargeting(
            src_human="xrobot",
            tgt_robot="unitree_g1",
            verbose=False,
        )
        print("[RetargetServer] Ready! Waiting for Unity input...", file=sys.stderr, flush=True)
    except Exception as e:
        print(f"[RetargetServer] FATAL: Failed to initialize GMR: {e}", file=sys.stderr, flush=True)
        sys.exit(1)

    # === 实时处理循环 ===
    frame_count = 0

    for line in sys.stdin:
        try:
            # === 1. Parse JSON from Unity ===
            data = json.loads(line.strip())

            # 数据格式:
            # {
            #   "positions": [[x,y,z], ...],  // 24个Vector3
            #   "rotations": [[x,y,z,w], ...]  // 24个Quaternion (Unity xyzw)
            # }

            positions = data["positions"]  # 24 x [x, y, z]
            rotations = data["rotations"]  # 24 x [x, y, z, w] (Unity xyzw)

            # === 2. 转换为GMR格式 ===
            frame = {}
            for i, name in enumerate(BODY_JOINT_NAMES):
                pos = positions[i]
                rot_xyzw = rotations[i]
                rot_wxyz = unity_quat_to_wxyz(rot_xyzw)

                frame[name] = [pos, rot_wxyz]

            # === 3. Unity→右手坐标系转换 ===
            frame = coordinate_transform_unity_data(frame)

            # === 4. Retarget ===
            qpos = gmr.retarget(frame)

            # === 5. 输出结果到Unity ===
            result = {
                "root_pos": qpos[:3].tolist(),
                "root_rot_wxyz": qpos[3:7].tolist(),  # MuJoCo wxyz
                "dof_pos": qpos[7:].tolist(),
            }

            # 使用print输出到stdout（Unity会读取）
            print(json.dumps(result), flush=True)

            frame_count += 1

            # 每100帧报告一次状态
            if frame_count % 100 == 0:
                print(f"[RetargetServer] Processed {frame_count} frames", file=sys.stderr, flush=True)

        except json.JSONDecodeError as e:
            print(f"[RetargetServer] JSON parse error: {e}", file=sys.stderr, flush=True)
        except KeyError as e:
            print(f"[RetargetServer] Missing key in data: {e}", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"[RetargetServer] Error processing frame {frame_count}: {e}", file=sys.stderr, flush=True)
            import traceback
            traceback.print_exc(file=sys.stderr)

    print(f"[RetargetServer] Shutting down. Total frames processed: {frame_count}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
