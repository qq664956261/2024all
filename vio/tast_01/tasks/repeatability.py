import torch
import numpy as np
import cv2
from utils.projection import warp
from utils.extracter import detection
from utils.visualization import plot_kps_error, write_txt, plot_matches


def mutual_argmax(value, mask=None, as_tuple=True):
    """
    Args:
        value: MxN
        mask:  MxN

    Returns:

    """
    value = value - value.min()  # convert to non-negative tensor
    if mask is not None:
        value = value * mask

    max0 = value.max(dim=1, keepdim=True)  # the col index the max value in each row
    max1 = value.max(dim=0, keepdim=True)

    valid_max0 = value == max0[0]
    valid_max1 = value == max1[0]

    mutual = valid_max0 * valid_max1
    if mask is not None:
        mutual = mutual * mask

    return mutual.nonzero(as_tuple=as_tuple)


def mutual_argmin(value, mask=None):
    return mutual_argmax(-value, mask)


def compute_keypoints_distance(kpts0, kpts1, p=2):
    """
    Args:
        kpts0: torch.tensor [M,2]
        kpts1: torch.tensor [N,2]
        p: (int, float, inf, -inf, 'fro', 'nuc', optional): the order of norm

    Returns:
        dist, torch.tensor [N,M]
    """
    dist = kpts0[:, None, :] - kpts1[None, :, :]  # [M,N,2]
    dist = torch.norm(dist, p=p, dim=2)  # [M,N]
    return dist


def val_key_points(kps0, kps1, desc0, desc1, warp01, warp10, th: int = 3):
    num_feat = min(kps0.shape[0], kps1.shape[0])

    # match
    matches_est = mutual_argmax(desc0.t() @ desc1)
    mkpts0, mkpts1 = kps0[matches_est[0]], kps1[matches_est[1]]
    mkpts0, mkpts01, ids0, _ = warp(mkpts0, warp01)
    dist = torch.sqrt(((mkpts01 - mkpts1[ids0, :2]) ** 2).sum(axis=1))
    if dist.shape[0] == 0:
        dist = dist.new_tensor([float('inf')])
    if 'resize' in warp01:
        dist = dist * warp01['resize']
    else:
        dist = dist * warp01['width']
    num_inlier = (dist <= th).sum().cpu()

    # ==================================== covisible keypoints
    kps0_cov, kps01_cov, _, _ = warp(kps0, warp01)
    kps1_cov, kps10_cov, _, _ = warp(kps1, warp10)
    num_cov_feat = (len(kps0_cov) + len(kps1_cov)) / 2  # number of covisible keypoints
    if kps0_cov.shape[0] == 0 or kps1_cov.shape[0] == 0:
        return {
            'num_feat': 0,
            'repeatability': 0,
            'matching_score': 0,
            'mean_error': 0,
            'errors': None,
            'matches_est': None,
        }
    # ==================================== get gt matching keypoints
    dist01 = compute_keypoints_distance(kps0_cov, kps10_cov)
    dist10 = compute_keypoints_distance(kps1_cov, kps01_cov)
    dist_mutual = (dist01 + dist10.t()) / 2.
    imutual = torch.arange(min(dist_mutual.shape), device=dist_mutual.device)
    dist_mutual[imutual, imutual] = 99999  # mask out diagonal
    mutual_min_indices = mutual_argmin(dist_mutual)
    dist = dist_mutual[mutual_min_indices]
    if 'resize' in warp01:
        dist = dist * warp01['resize']
        dist_mutual = dist_mutual * warp10['resize']
    else:
        dist = dist * warp01['width']
        dist_mutual = dist_mutual * warp10['width']
    gt_num = (dist <= th).sum().cpu()  # number of gt matching keypoints
    error = dist[dist <= th].cpu().numpy()
    mean_error = error.mean()
    errors = torch.min(dist_mutual, dim=1)[0]

    return {
        'num_feat': num_feat,
        'repeatability': gt_num / num_feat,
        'matching_score': num_inlier / max(num_cov_feat, 1),
        'mean_error': mean_error,
        'errors': errors,
        'matches_est': matches_est,
    }


def repeatability(idx, img_0, score_map_0, img_1, score_map_1, desc_map_0, desc_map_1, warp01, warp10, params):
    """
    Args:
        idx: int
        img_0: torch.tensor [H,W,3]
        score_map_0: torch.tensor [H,W]
        img_1: torch.tensor [H,W,3]
        score_map_1: torch.tensor [H,W]
        warp01: dict
        warp10: dict
        params: int
    Returns:
        dict
    """

    # 1. detection
    kps0 = detection(score_map_0, params['extractor_params'])
    kps1 = detection(score_map_1, params['extractor_params'])

    kps0_ = kps0[:, :2] * 2 - 1
    kps1_ = kps1[:, :2] * 2 - 1
    desc0 = torch.nn.functional.grid_sample(desc_map_0, kps0_.unsqueeze(0).unsqueeze(0),
                                            mode='bilinear', align_corners=True)[0, :, 0, :]  # CxN
    desc0 = torch.nn.functional.normalize(desc0, p=2, dim=0)
    desc1 = torch.nn.functional.grid_sample(desc_map_1, kps1_.unsqueeze(0).unsqueeze(0),
                                            mode='bilinear', align_corners=True)[0, :, 0, :]  # CxN
    desc1 = torch.nn.functional.normalize(desc1, p=2, dim=0)

    # 2. validation
    result = val_key_points(kps0, kps1, desc0, desc1, warp01, warp10, th=params['repeatability_params']['th'])
    # 3. save image
    show = plot_kps_error(img_0, kps0, result['errors'], params['repeatability_params']['image'])
    root = params['repeatability_params']['output']
    cv2.imwrite(root + str(idx) + '_repeatability_0.png', show)
    show = plot_kps_error(img_1, kps1, None, params['repeatability_params']['image'])
    cv2.imwrite(root + str(idx) + '_repeatability_1.png', show)
    # add
    _, _, h, w = img_0.shape
    kps0_hw = kps0[result['matches_est'][0], 0:2] * torch.tensor([w - 1, h - 1]).to(kps0.device)
    kps1_hw = kps1[result['matches_est'][1], 0:2] * torch.tensor([w - 1, h - 1]).to(kps0.device)
    show = plot_matches(img_0, img_1, kps0_hw, kps1_hw)
    cv2.imwrite(root + str(idx) + '_matches.png', show)
    return result


def plot_repeatability(repeatability, save_path):
    import matplotlib.pyplot as plt
    plt.plot(repeatability)
    plt.savefig(save_path)
    plt.close()
    write_txt(save_path.replace('.png', '.txt'), repeatability)
