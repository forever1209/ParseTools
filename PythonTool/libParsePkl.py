import pickle
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_sensor_tran_matrix(quats, trans):
    """
    外参转换为矩阵形式
    """
    # trans = np.array([trans[key] for key in ["x", "y", "z"]], dtype=np.float32)
    # quats = np.array([quats[key] for key in ["x", "y", "z", "w"]], dtype=np.float32)

    rotation = R.from_quat(quats).as_matrix()
    trans_matrix = np.eye(4, dtype=np.float32)
    trans_matrix[:3, :3] = rotation
    trans_matrix[:3, 3] = trans
    return trans_matrix
class PklParser:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = None
    def get_data(self):
        try:
            with open(self.file_path, 'rb') as f:
                data = pickle.load(f)
            return data
        except FileNotFoundError:
            print("File not found:", self.file_path)
            return None
        except Exception as e:
            print("An error occurred while parsing the pickle file:", e)
            return None
    def create_frustum(self,final_dim, downsample_factor, d_bound):
        """Generate frustum"""
        # make grid in image plane
        ogfH, ogfW = final_dim
        fH, fW = ogfH // downsample_factor, ogfW // downsample_factor
        d_coords = np.tile(np.arange(*d_bound, dtype=np.float32).reshape(-1, 1, 1), (1, fH, fW))
        D, _, _ = d_coords.shape
        x_coords = np.tile(
            np.arange(
                downsample_factor / 2, ogfW + downsample_factor / 2, downsample_factor, dtype=np.float32
            )
            .reshape(1, 1, fW), (D, fH, 1)
        )
        y_coords = np.tile(
            np.arange(
                downsample_factor / 2, ogfH + downsample_factor / 2, downsample_factor, dtype=np.float32
            )
            .reshape(1, fH, 1), (D, 1, fW)
        )
        paddings = np.ones_like(d_coords)

        # D x H x W x 3
        frustum = np.stack((x_coords, y_coords, d_coords, paddings), -1)
        return frustum
    def get_geometry(self, intrinsic, extrinsic, ida_mat, voxel_size, voxel_coord, voxel_num, frustum , path_name):
        batch_size = 1
        num_cams = intrinsic.shape[0]
        intrin_mats = np.eye(4, dtype=np.float32).reshape(1,4,4).repeat(num_cams,axis=0)
        intrin_mats[:, :3, :3] = intrinsic
        tgt2imgs = intrin_mats @ extrinsic
        # undo post-transformation
        # B x N x D x H x W x 3
        points = frustum

        ida_mat = ida_mat.reshape(batch_size, num_cams, 1, 1, 1, 4, 4)
        points = np.linalg.inv(ida_mat) @ (points[..., None])

        # camera to lidar
        points = points.squeeze(-1)
        points = np.concatenate(
            (
                points[:, :, :, :, :, :2] * points[:, :, :, :, :, 2:3],
                points[:, :, :, :, :, 2:3],
                np.ones((*points.shape[:5], 1), dtype=points.dtype),
            ),
            5,
        )  # (B, N, D, H, W, 4)
        points = (
            np.linalg.inv(tgt2imgs).reshape(batch_size, num_cams, 1, 1, 1, 4, 4) @ points[..., None]).squeeze(-1)
        points = points[..., :3]
        # voxel_size, voxel_coord, voxel_num = voxel_cfg
        points = ((points - (voxel_coord - voxel_size / 2.0)) / voxel_size).astype(int)
        # print("points shape is ",points.shape)
        if "geom_lane" in path_name:
            points[..., :2] = voxel_num[:2] - points[..., :2] - 1
            points = points[..., [1, 0, 2]]
            np.save(path_name, points)
        else:
            np.save(path_name, points)
        return True
#test
# aa = PklParser("/home/liulei10/下载/fovs_intrinsic_k_dict.pkl")
# data = aa.get_data()
# new_K_120 = data["fov30"]
# print('before ',new_K_120)
# print('\n')
# new_K_120[:2] = new_K_120[:2] / 2
# print('new_K_120 \n ',new_K_120)