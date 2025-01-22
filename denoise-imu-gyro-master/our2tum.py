import os
import pickle
import numpy as np
import torch
from scipy.interpolate import interp1d
from src.lie_algebra import SO3

def load_imu_data(log_file):
    """
    从日志文件中读取IMU数据。
    :param log_file: IMU日志文件路径
    :return: numpy数组，包含时间戳和IMU数据
    """
    imu_data = []
    with open(log_file, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 11:  # 确保数据行包含所有列
                imu_timestamp, index, roll, pitch, yaw, wx, wy, wz, ax, ay, az = map(float, parts)
                
                # 转换单位
                ax = ax / 1000.0 * 9.81
                ay = ay / 1000.0 * 9.81
                az = az / 1000.0 * 9.81
                wx = wx / 100.0 * np.pi / 180.0
                wy = wy / 100.0 * np.pi / 180.0
                wz = wz / 100.0 * np.pi / 180.0
                roll = roll / 100.0 * np.pi / 180.0
                pitch = pitch / 100.0 * np.pi / 180.0
                yaw = yaw / 100.0 * np.pi / 180.0
                #print(az)
                
                # 保存为一行数据
                imu_data.append([imu_timestamp, roll, pitch, yaw, wx, wy, wz, ax, ay, az])
    return np.array(imu_data)  # 确保返回的是二维数组

def align_and_interpolate(imu_data, gt_data):
    """
    对齐IMU数据和地面真值，并将地面真值插值到IMU时间戳上。
    :param imu_data: IMU数据数组 (Nx10, 第一列为时间戳)
    :param gt_data: 地面真值数组 (Mx8, 第一列为时间戳)
    :return: 对齐和插值后的IMU数据和地面真值
    """
    # 提取时间戳
    imu_timestamps = imu_data[:, 0]
    gt_timestamps = gt_data[:, 0]

    # 对齐时间范围
    t_start = max(imu_timestamps[0], gt_timestamps[0])
    t_end = min(imu_timestamps[-1], gt_timestamps[-1])

    imu_mask = (imu_timestamps >= t_start) & (imu_timestamps <= t_end)
    gt_mask = (gt_timestamps >= t_start) & (gt_timestamps <= t_end)

    imu_data = imu_data[imu_mask]
    gt_data = gt_data[gt_mask]

    imu_timestamps = imu_data[:, 0]
    gt_timestamps = gt_data[:, 0]

    # 插值地面真值到IMU时间戳
    gt_positions = interp1d(gt_timestamps, gt_data[:, 1:4], axis=0, fill_value="extrapolate")(imu_timestamps)
    gt_quaternions = interp1d(gt_timestamps, gt_data[:, 4:8], axis=0, fill_value="extrapolate")(imu_timestamps)

    # 将插值的四元数归一化
    gt_quaternions = gt_quaternions / np.linalg.norm(gt_quaternions, axis=1, keepdims=True)

    return imu_data, gt_positions, gt_quaternions

def save_as_tumvi_format(output_file, imu_data, gt_positions, gt_quaternions):
    """
    保存为TUMVI数据集格式的`.p`文件。
    :param output_file: 输出文件路径
    :param imu_data: IMU数据
    :param gt_positions: 地面真值位置
    :param gt_quaternions: 地面真值四元数
    """
    # IMU数据
    imu_timestamps = imu_data[:, 0]
    imu_measurements = torch.tensor(imu_data[:, 4:], dtype=torch.float32)  # 加速度+角速度

    # 地面真值数据
    positions = torch.tensor(gt_positions, dtype=torch.float32)
    quaternions = torch.tensor(gt_quaternions, dtype=torch.float32)
    velocities = torch.zeros_like(positions)
    velocities[1:] = (positions[1:] - positions[:-1]) / (imu_timestamps[1:] - imu_timestamps[:-1]).reshape(-1, 1)

    # 保存为字典格式
    data_dict = {
        "ts": imu_timestamps,
        "qs": quaternions,
        "vs": velocities,
        "ps": positions,
    }

        # 计算旋转增量 (xs)
    gt_quaternions_tensor = torch.tensor(gt_quaternions, dtype=torch.float32)
    dRot_ij = SO3.qmul(SO3.qinv(gt_quaternions_tensor[:-1]), gt_quaternions_tensor[1:])
    xs = SO3.qlog(dRot_ij)  # 从旋转增量计算对数映射

    # 保存为字典格式
    data_dict = {
        "xs": xs.float(),
        "us": imu_measurements.float(),  # IMU原始测量值
    }

    with open(output_file, "wb") as f:
        pickle.dump(data_dict, f)
    print(f"Saved to {output_file}")

if __name__ == "__main__":
    # 输入文件路径
    imu_log_file = "./imu.log"  # 替换为你的IMU日志文件路径
    gt_file = "/path/to/ground_truth.csv"  # 替换为地面真值文件路径
    output_file = "./dataset-our1.p"

    # 加载IMU和地面真值数据
    imu_data = load_imu_data(imu_log_file)
    #gt_data = np.genfromtxt(gt_file, delimiter=",", skip_header=1)  # 假设地面真值是CSV格式

    # 对齐和插值
    imu_data, gt_positions, gt_quaternions = align_and_interpolate(imu_data, imu_data)

    # 保存为TUMVI格式
    save_as_tumvi_format(output_file, imu_data, gt_positions, gt_quaternions)
