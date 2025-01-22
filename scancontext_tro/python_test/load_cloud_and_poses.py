import numpy as np
import open3d as o3d
import transformations as tf
import math
import copy

def load_point_cloud_from_bin(bin_path):
    """从.bin文件加载点云"""
    point_cloud = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 3)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(point_cloud[:, :3])  # 假设前三列是x,wy,z坐标
    return pc

def quatation_to_matrix(pose_vector):
    wx, wy, wz, w = pose_vector[-4:]
    rotation = np.array([
        [1 - 2 * wy * wy - 2 * wz * wz, 2 * (wx * wy - wz * w), 2 * (wx * wz + wy * w)],
        [2 * (wx * wy + wz * w), 1 - 2 * wx * wx - 2 * wz * wz, 2 * (wy * wz - wx * w)],
        [2 * (wx * wz - wy * w), 2 * (wy * wz + wx * w), 1 - 2 * wx * wx - 2 * wy * wy]])

    # print("yaw: ", yaw, " pitch: ", pitch, " roll: ", roll)
    translation = pose_vector[:3].reshape([3, 1])
    matrix = np.concatenate([rotation, translation], axis=1)
    expand_item = np.array([0, 0, 0, 1]).reshape((1, 4))
    matrix = np.concatenate([matrix, expand_item], axis=0)
    
    return matrix

def quaternion_to_euler(quaternion):
    x, y, z, w = quaternion
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

def euler_to_rotation(roll_rad, pitch_rad, yaw_rad):
    # 将欧拉角转换为弧度
    # roll_rad = math.radians(roll)
    # pitch_rad = math.radians(pitch)
    # yaw_rad = math.radians(yaw)

    # 计算旋转矩阵的元素
    cos_r = math.cos(roll_rad)
    sin_r = math.sin(roll_rad)
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)

    # 构建旋转矩阵
    rotation = np.array([[cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
                         [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
                         [-sin_p, cos_p * sin_r, cos_p * cos_r]])

    return rotation

def pose_vector3d_to_matrix_pose_2d(pose_vector):
    euler = tf.euler_from_quaternion(pose_vector[-4:])
    x, y = pose_vector[0:2]
    rotation_matrix = tf.euler_matrix(euler[0], 0, 0, 'szyx')
    # print("origin, x, y, yaw: ", x, y, euler[0])
    euler_new = tf.euler_from_matrix(rotation_matrix)
    # print("euler_new: ", euler_new)

    translation = pose_vector[:3].reshape([3, 1])
    rotation_matrix[0:3, 3:4] = translation
    # matrix = np.concatenate([rotation_matrix, translation], axis=1)
    # expand_item = np.array([0, 0, 0, 1]).reshape((1, 4))
    # matrix = np.concatenate([matrix, expand_item], axis=0)
    return rotation_matrix

def load_poses_from_txt(pose_path):
    """从txt文件加载位姿矩阵"""
    poses = []
    with open(pose_path, 'r') as file:
        for line in file:
            pose_vector = np.fromstring(line, sep=' ')[1:]
            pose_2d = pose_vector3d_to_matrix_pose_2d(pose_vector)
            
            poses.append(pose_2d)
    return poses

def transform_point_cloud(pc, poses):
    """将点云根据位姿列表转换到不同的坐标系下，并返回转换后的点云列表"""
    transformed_pcs = []
    for pose in poses:
        temp_psc = copy.deepcopy(pc)
        temp_psc.transform(np.linalg.inv(pose))
        transformed_pcs.append(temp_psc)
    return transformed_pcs

def get_transformed_point_cloud(point_cloud_bin_path, poses_path):

    # 加载点云和位姿
    point_cloud = load_point_cloud_from_bin(point_cloud_bin_path)
    poses = load_poses_from_txt(poses_path)
    
    # 转换点云
    transformed_point_clouds = transform_point_cloud(point_cloud, poses)
    
    # 这里可以添加代码来处理或可视化转换后的点
    # 例如，可视化source和target点云的第一个转换结果asdf
    # origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0,0,0])
    # o3d.visualization.draw_geometries([origin, transformed_point_cloud[0]])
    # o3d.io.write_point_cloud("test0.pcd", transformed_point_cloud[0])
    return transformed_point_clouds

def get_transform_maxtrix_from_yaw(yaw):
    yaw_rad = np.radians(yaw)
    R = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
              [np.sin(yaw_rad), np.cos(yaw_rad), 0],
              [0, 0, 1]])
    R = np.linalg.inv(R)
    return R

if __name__ == '__main__':
    from make_sc_example_modify import ScanContext
    from Distance_SC import distance_sc
    import time
    import tqdm
    # 输入文件路径
    source_bin_path = '/home/zc/data/04_relocation/0319_1819_0229/source/map/point_cloud_source.bin'
    target_bin_path = '/home/zc/data/04_relocation/0319_1819_0229/target/map/point_cloud_1819.bin'
    source_pose_path = '/home/zc/data/04_relocation/0319_1819_0229/source/hj_slam_pose.txt'
    target_pose_path = '/home/zc/data/04_relocation/0319_1819_0229//target/hj_slam_pose.txt'
    
    source_point_clouds = get_transformed_point_cloud(source_bin_path, source_pose_path)
    target_point_clouds = get_transformed_point_cloud(target_bin_path, target_pose_path)

    source_size = len(source_point_clouds)
    target_size = len(target_point_clouds)
    for i in range(1000, source_size, 200):
        iterms = []
        start = time.time()
        for j in tqdm.tqdm(range(0, target_size, 5)):
            # j = 0
            source_sc = ScanContext(source_point_clouds[i])
            target_sc = ScanContext(target_point_clouds[j])
            # source_sc.plot_multiple_sc(1)
            # target_sc.plot_multiple_sc(1)
            source_SC = source_sc.SCs[0]
            target_SC = target_sc.SCs[0]
            distance, roll_index = distance_sc(target_SC, source_SC)

            print("distance: ", distance, " i: ", i, " j: ", j)
            iterms.append([distance, i, j, roll_index])
        end = time.time()
        print("duration: ", end - start)
        import matplotlib.pyplot as plt
        distances = np.array([iterm[0] for iterm in iterms])
        min_index = np.argmin(distances)
        print("min distance: ", iterms[min_index])
        plt.plot(distances)
        plt.show()

        _, source_index, target_index, roll_index = iterms[min_index]
        origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2, origin=[0,0,0])

        source_point_cloud_result = source_point_clouds[source_index]
        target_point_cloud_result = target_point_clouds[target_index]

        yaw =  (60 - iterms[min_index][3] - 1) * 6
        print("yaw: ", yaw)
        o3d.visualization.draw_geometries([origin, source_point_cloud_result, target_point_cloud_result])
        o3d.io.write_point_cloud("source_before.pcd", source_point_cloud_result)
        o3d.io.write_point_cloud("target_before.pcd", target_point_cloud_result)
        # R = get_transform_maxtrix_from_yaw(yaw)
        # target_point_cloud_result.rotate(R)
        # o3d.visualization.draw_geometries([origin, source_point_cloud_result])
        # o3d.visualization.draw_geometries([origin, target_point_cloud_result])
        # o3d.visualization.draw_geometries([origin, source_point_cloud_result, target_point_cloud_result])
        # o3d.io.write_point_cloud("source.pcd", source_point_cloud_result)
        # o3d.io.write_point_cloud("target.pcd", target_point_cloud_result)
