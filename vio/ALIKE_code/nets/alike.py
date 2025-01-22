import cv2
import math
import torch
import logging
from copy import deepcopy
from torchvision.transforms import ToTensor
import sys
sys.path.append('..')  # 添加上级目录到模块搜索路径中
from nets.alnet import ALNet
from nets.soft_detect import SoftDetect
import time

# 1. ALNet 基类
# ALNet 是一个特征提取器网络，用于从输入图像中提取特征图。这个网络分为多个卷积块（如 block1, block2, block3, block4），这些块通过层级结构从图像中提取不同尺度的特征。
# agg_mode 决定了特征图如何聚合
# 2. ALIKE 类
# ALIKE 继承自 ALNet，并在其基础上添加了特征点检测和描述子提取的功能。
class ALIKE(ALNet):
    def __init__(self,
                 # ================================== feature encoder
                #  c1, c2, c3, c4, dim:
                # 这些参数用于指定网络的卷积层的通道数和特征维度。
                # c1, c2, c3, c4 分别是网络中不同层的卷积核通道数。
                # dim 是特征描述子的维度。
                 c1: int = 32, c2: int = 64, c3: int = 128, c4: int = 128, dim: int = 128,
                #  特征聚合模式，决定如何将不同尺度的特征图进行合并
                 agg_mode: str = 'cat',  # sum, cat, fpn
                #  布尔值，决定是否使用单个头（True）或两个头（False）。这影响到最后的卷积头数和输出。
                 single_head: bool = False,
                #  决定是否使用位置编码（Position Encoding）。如果为 True，会在特征图中添加额外的位置信息。
                 pe: bool = False,
                 # ================================== detect parameters
                #  用于特征点检测的半径，影响特征点的邻域范围
                 radius: int = 2,
                #  选择前 top_k 个特征点，以确保提取的特征点数目。
                # 分数阈值，仅保留分数高于该值的特征点。
                 top_k: int = 500, scores_th: float = 0.5,
                #  特征点的最大数量限制。设为 0 时表示没有限制。
                 n_limit: int = 0,
                 **kwargs
                 ):
        # ALNet 类的构造函数
        # super(): 这个函数用于调用当前类的父类（基类）的构造函数。
        # __init__: 是 Python 类中的构造函数方法，用于初始化类的实例。
        super().__init__(c1, c2, c3, c4, dim, agg_mode, single_head, pe)

        self.radius = radius
        self.top_k = top_k
        self.n_limit = n_limit
        self.scores_th = scores_th

        self.update_softdetect_parameters()

    def update_softdetect_parameters(self):
        self.softdetect = SoftDetect(radius=self.radius, top_k=self.top_k,
                                     scores_th=self.scores_th, n_limit=self.n_limit)

    def extract_dense_map(self, image, ret_dict=False):
        # ====================================================
        # check image size, should be integer multiples of 2^5
        # if it is not a integer multiples of 2^5, padding zeros
        device = image.device
        b, c, h, w = image.shape
        h_ = math.ceil(h / 32) * 32 if h % 32 != 0 else h
        w_ = math.ceil(w / 32) * 32 if w % 32 != 0 else w
        if h_ != h:
            h_padding = torch.zeros(b, c, h_ - h, w, device=device)
            image = torch.cat([image, h_padding], dim=2)
        if w_ != w:
            w_padding = torch.zeros(b, c, h_, w_ - w, device=device)
            image = torch.cat([image, w_padding], dim=3)
        # ====================================================

        scores_map, descriptor_map = super().forward(image)

        # ====================================================
        if h_ != h or w_ != w:
            descriptor_map = descriptor_map[:, :, :h, :w]
            scores_map = scores_map[:, :, :h, :w]  # Bx1xHxW
        # ====================================================

        # BxCxHxW
        descriptor_map = torch.nn.functional.normalize(descriptor_map, p=2, dim=1)

        if ret_dict:
            return {'descriptor_map': descriptor_map, 'scores_map': scores_map, }
        else:
            return descriptor_map, scores_map

    def extract(self, image):
        descriptor_map, scores_map = self.extract_dense_map(image)
        keypoints, descriptors, kptscores, scoredispersitys = self.softdetect(scores_map, descriptor_map)

        return {'keypoints': keypoints,  # B M 2
                'descriptors': descriptors,  # B M D
                'scores': kptscores,  # B M D
                'score_dispersity': scoredispersitys,
                'descriptor_map': descriptor_map,  # BxCxHxW
                'scores_map': scores_map,  # Bx1xHxW
                }

    def forward(self, img, scale_f=2 ** 0.5,
                min_scale=1., max_scale=1.,
                min_size=0., max_size=99999.,
                image_size_max=99999,
                verbose=False, n_k=0, sort=False,
                scoremap=True,
                descmap=True):
        """
        :param img: np array, HxWx3
        :param scale_f:
        :param min_scale:
        :param max_scale:
        :param min_size:
        :param max_size:
        :param verbose:
        :param n_k:
        :param sort:
        :return: keypoints, descriptors, scores
        """
        old_bm = torch.backends.cudnn.benchmark
        torch.backends.cudnn.benchmark = False  # speedup

        H_, W_, three = img.shape
        assert three == 3, "input image shape should be [HxWx3]"

        # ==================== image size constraint
        image = deepcopy(img)
        max_hw = max(H_, W_)
        if max_hw > image_size_max:
            ratio = float(image_size_max / max_hw)
            image = cv2.resize(image, dsize=None, fx=ratio, fy=ratio)

        # ==================== convert image to tensor
        H, W, three = image.shape
        image = ToTensor()(image).unsqueeze(0)
        image = image.to(self.device)

        # ==================== extract keypoints at multiple scales
        start = time.time()

        s = 1.0  # current scale factor
        if verbose:
            logging.info('')
        keypoints, descriptors, scores, scores_maps, descriptor_maps = [], [], [], [], []
        while s + 0.001 >= max(min_scale, min_size / max(H, W)):
            if s - 0.001 <= min(max_scale, max_size / max(H, W)):
                nh, nw = image.shape[2:]

                # extract descriptors
                with torch.no_grad():
                    descriptor_map, scores_map = self.extract_dense_map(image)
                    keypoints_, descriptors_, scores_, _ = self.softdetect(scores_map, descriptor_map)

                if scoremap:
                    scores_maps.append(scores_map[0, 0].cpu())
                if descmap:
                    descriptor_maps.append(descriptor_map[0].cpu())
                keypoints.append(keypoints_[0])
                descriptors.append(descriptors_[0])
                scores.append(scores_[0])

                if verbose:
                    logging.info(
                        f"extracting at scale x{s:.02f} = {nw:4d}x{nh:3d}. Number of keypoints {len(keypoints)}.")

            s /= scale_f

            # down-scale the image for next iteration
            nh, nw = round(H * s), round(W * s)
            image = torch.nn.functional.interpolate(image, (nh, nw), mode='bilinear', align_corners=False)

        # restore value
        torch.backends.cudnn.benchmark = old_bm

        keypoints = torch.cat(keypoints)
        descriptors = torch.cat(descriptors)
        scores = torch.cat(scores)
        keypoints = (keypoints + 1) / 2 * keypoints.new_tensor([[W_ - 1, H_ - 1]])

        if sort or 0 < n_k < len(keypoints):
            indices = torch.argsort(scores, descending=True)
            keypoints = keypoints[indices]
            descriptors = descriptors[indices]
            scores = scores[indices]

        if 0 < n_k < len(keypoints):
            keypoints = keypoints[0:n_k]
            descriptors = descriptors[0:n_k]
            scores = scores[0:n_k]

        return {'keypoints': keypoints, 'descriptors': descriptors, 'scores': scores,
                'descriptor_maps': descriptor_maps,
                'scores_maps': scores_maps, 'time': time.time() - start, }


if __name__ == '__main__':
    import numpy as np
    from thop import profile

    net = ALIKE(c1=32, c2=64, c3=128, c4=128, dim=128, agg_mode='cat', res_block=True, single_head=False)

    image = np.random.random((640, 480, 3)).astype(np.float32)
    flops, params = profile(net, inputs=(image, 2 ** 0.25, 0.3, 1, 0, 9999, 9999, False, 5000, False))
    print('{:<30}  {:<8} GFLops'.format('Computational complexity: ', flops / 1e9))
    print('{:<30}  {:<8} KB'.format('Number of parameters: ', params / 1e3))
