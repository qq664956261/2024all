import torch
from torch import nn
import numpy as np
# from pytorch_lightning.core.lightning import LightningModule
from pytorch_lightning import LightningModule
import torch.nn.functional as F


# coordinates system
#  ------------------------------>  [ x: range=-1.0~1.0; w: range=0~W ]
#  | -----------------------------
#  | |                           |
#  | |                           |
#  | |                           |
#  | |         image             |
#  | |                           |
#  | |                           |
#  | |                           |
#  | |---------------------------|
#  v
# [ y: range=-1.0~1.0; h: range=0~H ]
# 函数实现了快速的非极大值抑制（Non-Maximum Suppression, NMS）算法，用于去除图像中邻近的点。
# NMS 是在许多计算机视觉任务中广泛使用的一种技术，特别是在检测关键点或对象时，用于保留强响应点并抑制邻近较弱的响应点。
# 接受两个参数：scores（得分图）和 nms_radius（非极大值抑制的半径）。
def simple_nms(scores, nms_radius: int):
    """ Fast Non-maximum suppression to remove nearby points """
    assert (nms_radius >= 0)
    # 在 max_pool 函数中，对输入张量 x 进行 2D 最大池化操作。
    # kernel_size 为 (nms_radius * 2 + 1)，即在给定的 nms_radius 范围内计算最大值。
    # stride=1 表示每次移动一个像素。
    # padding=nms_radius 确保边界上的像素也会被考虑。
    def max_pool(x):
        return torch.nn.functional.max_pool2d(
            x, kernel_size=nms_radius * 2 + 1, stride=1, padding=nms_radius)
    # 创建一个与 scores 大小相同的全零张量，用于后续操作。
    zeros = torch.zeros_like(scores)
    # max_pool(scores) 返回一个池化后的张量，其中每个位置的值是其周围 nms_radius 范围内的最大值。
    # scores == max_pool(scores) 会生成一个布尔掩码，表示哪些位置的得分等于该位置周围的最大值，这些位置将被保留。
    max_mask = scores == max_pool(scores)

    for _ in range(2):
        # 操作分解
        # max_mask.float()
        # 这里将布尔掩码 max_mask 转换为浮点数类型。布尔值 True 会变成 1.0，False 会变成 0.0。
        # 这样做的目的是为了方便后续的数值运算，因为 max_pool 操作需要数值输入。
        # max_pool(max_mask.float())
        # 这个操作对 max_mask.float() 进行最大池化。最大池化是一个滑动窗口操作，它在每个窗口内取最大值。
        # 因为 max_mask.float() 只有 0.0 和 1.0，所以这个操作会在每个滑动窗口内检查是否存在 1.0（即 True），如果存在，输出的池化结果就会是 1.0；如果不存在，池化结果就会是 0.0。
        # > 0
        # 这一部分将最大池化的结果转换回布尔值。
        # 如果某个位置的池化结果大于 0（即在池化窗口内存在 1.0），那么输出的 supp_mask 在该位置为 True；否则为 False。
        # 这一行代码的整体作用是生成一个掩码 supp_mask，标记那些在 nms_radius 半径范围内存在局部最大值的位置。
        # 这意味着在这个范围内的所有局部最大值点都会被标记为 True。这样可以进一步扩展抑制的范围，
        # 使得那些离局部最大值比较近的其他点也能被抑制掉，从而只保留最显著的局部最大值。
        # 假设 max_mask 是以下 5x5 的矩阵，其中 1 表示局部最大值，0 表示其他位置：
        # 0 0 0 0 0
        # 0 1 0 0 0
        # 0 0 0 1 0
        # 0 0 0 0 0
        # 0 0 0 0 0
        # 假设 nms_radius = 1，那么 max_pool(max_mask.float()) 会生成一个 3x3 滑动窗口，来检查每个位置的最大值。结果会是：
        # 1 1 1 0 0
        # 1 1 1 1 1
        # 1 1 1 1 1
        # 0 1 1 1 0
        # 0 0 0 0 0
        # 在这个矩阵中，1 表示在该位置及其邻域内存在局部最大值（即至少有一个 True）。这就是 supp_mask。
        supp_mask = max_pool(max_mask.float()) > 0
        # 作用
        # 目的：这一行代码的主要目的是通过 supp_mask 抑制掉与局部最大值接近的其他得分点，确保这些位置不会再被认为是新的局部最大值。
        # 具体操作：通过 supp_mask 将所有在 nms_radius 半径内（或局部范围内）与局部最大值接近的得分点设置为 0，以防止它们干扰下一步的非极大值抑制过程。
        # torch.where(condition, x, y)
        # condition 是一个布尔张量，在这里是 supp_mask。
        # x 是当 condition 为 True 时使用的值，在这里是 zeros。
        # y 是当 condition 为 False 时使用的值，在这里是 scores。
        # 工作机制
        # 对于 supp_mask 中为 True 的位置，torch.where 会在 supp_scores 中将这些位置的值设置为 zeros（即抑制这些位置的得分）。
        # 对于 supp_mask 中为 False 的位置，torch.where 会在 supp_scores 中保留原来的 scores 值（即这些位置的得分保持不变）。
        supp_scores = torch.where(supp_mask, zeros, scores)
        # new_max_mask 生成一个新的掩码，表示哪些位置在当前抑制步骤后仍然是局部极大值。这个掩码会在接下来的步骤中用于进一步更新最终的极大值掩码。
        new_max_mask = supp_scores == max_pool(supp_scores)
        max_mask = max_mask | (new_max_mask & (~supp_mask))
    return torch.where(max_mask, scores, zeros)


def sample_descriptor(descriptor_map, kpts):
    """
    :param descriptor_map: BxCxHxW  ->  表示批次（B）中的特征图，每个特征图的大小为 CxHxW（通道数、高度、宽度）。
    :param kpts: list, len=B, each is Nx2 (keypoints) [h,w]  ->  表示每张图片的关键点坐标 (h, w)，格式为批次大小的列表，每个元素为 Nx2 的张量。N：表示关键点的数量。对于每张图像，可能检测到多个关键点，N 是这些关键点的个数。
2：表示每个关键点的坐标，这两个值分别是 (x, y) 或 (h, w)，即关键点在图像中的水平和垂直位置。
    :return: descriptors: list, len=B, each is NxD  ->  返回描述符列表，其中每个元素的维度为 NxD（关键点数量 × 描述符维度）。
    """
    batch_size, channel, height, width = descriptor_map.shape

    descriptors = []
    for index in range(batch_size):
        kptsi = kpts[index]  # Nx2,(x,y)# Nx2, (x,y) 格式
        # 对于 grid_sample 函数来说，输入的特征图需要包含一个批次维度，通常是形状为 BxCxHxW。由于当前的 descriptor_map[index] 只包含 CxHxW，所以需要通过 unsqueeze(0) 将其转换为形状为 1xCxHxW，即在最前面添加一个批次维度 B=1。
        # grid_sample 是 PyTorch 中用于在特定坐标位置上采样特征图的函数。这里的目的是在每个关键点坐标位置（kptsi）上提取对应的特征向量。
        # grid_sample 会根据关键点坐标（kptsi.view(1, 1, -1, 2)）在特征图中进行双线性插值采样，得到对应的特征向量。
        # view(1, 1, -1, 2) 的作用：
        # 1, 1: 前两个维度表示批次和通道数，通常用于 grid_sample 的输入。
        # -1: 表示自动计算这一维度的大小。在这里，它将计算出 N，即关键点的数量。
        # 2: 表示每个关键点的两个坐标 (x, y)。
        # 假设 grid_sample 的输出张量的形状是 [1, C, 1, N]（其中 N 是关键点的数量）。[0, :, 0, :] 选择了这个张量的：
        # 批次中的第一张图像（0）。
        # 所有通道（:）。
        # 第一个输出高度位置（0）。
        # 所有宽度位置（:）。
        # 因此，[0, :, 0, :] 会选择一个形状为 [C, N] 的张量，其中 C 是通道数，N 是关键点的数量。这是因为我们在 grid_sample 中指定了 output_size 为 1xN 的特征图，选择了特定位置的所有通道
        descriptors_ = torch.nn.functional.grid_sample(descriptor_map[index].unsqueeze(0), kptsi.view(1, 1, -1, 2),
                                                       mode='bilinear', align_corners=True)[0, :, 0, :]  # CxN
        # input：要归一化的张量。在这里是 descriptors_，其形状为 [C, N]。
        # p：范数的类型。p=2 表示 L2 范数，即欧几里得范数。
        # dim：归一化操作应用的维度。dim=0 表示沿着第 0 维（通道维度）进行归一化。
        # 假设我们有一个二维张量（矩阵）A，其形状为 [3, 4]，代表有 3 行和 4 列。这里的维度可以表示为：

        # dim=0：纵向维度（列方向），每一列。
        # dim=1：横向维度（行方向），每一行。
        descriptors_ = torch.nn.functional.normalize(descriptors_, p=2, dim=0)
        # 为什么要转置？
        # 形状匹配：

        # 通常，深度学习框架中的特征描述符在处理时期望形状为 [N, C]，其中 N 是关键点的数量，C 是每个关键点的特征维度。因此，将 [C, N] 转置为 [N, C] 是为了符合这种期望。
        # 数据结构：

        # 在许多实现中，关键点和它们的描述符被组织成 [N, C] 的格式，其中 N 是关键点的数量，C 是每个关键点的特征维度。这种结构使得对每个关键点进行操作和存取变得更直观和一致。
        descriptors.append(descriptors_.t())

    return descriptors


class SoftDetect(LightningModule):
    def __init__(self, radius=2, top_k=0, scores_th=0.2, n_limit=20000):
        """
        Args:
            radius: soft detection radius, kernel size is (2 * radius + 1)
            top_k: top_k > 0: return top k keypoints
            scores_th: top_k <= 0 threshold mode:  scores_th > 0: return keypoints with scores>scores_th
                                                   else: return keypoints with scores > scores.mean()
            n_limit: max number of keypoint in threshold mode
        """
        super().__init__()
        self.radius = radius
        self.top_k = top_k
        self.scores_th = scores_th
        self.n_limit = n_limit
        self.kernel_size = 2 * self.radius + 1
        self.temperature = 0.1  # tuned temperature
        # torch.nn.Unfold 是 PyTorch 中的一个类，用于将图像张量的局部区域（patches）展平为更简单的形状。
        # 这对于某些操作（如卷积操作的实现）非常有用，因为它允许你将卷积的操作展平为矩阵乘法操作。
        # Unfold 的基本原理
        # 假设我们有一个大小为 B×C×H×W 的输入张量，其中：
        # B 是批次大小 (batch size)。
        # C 是通道数 (number of channels)。
        # H 和 W 分别是图像的高度和宽度。
        # Unfold 操作通过提取一个固定大小的窗口（卷积核）在输入张量上滑动，并将每个窗口中的数据展平为一个向量。最终，Unfold 的输出将是一个大小为 
        # B×(C × Kh × Kw)×L 的张量，其中：Kh和 Kw分别是卷积核的高度和宽度。
        # L 是滑动窗口在输入图像上滑动的次数，即窗口可以放置的所有位置的数量
        # x = torch.tensor([[[[1, 2, 3, 4],
        #             [5, 6, 7, 8],
        #             [9, 10, 11, 12],
        #             [13, 14, 15, 16]]]])
        # unfold = torch.nn.Unfold(kernel_size=(2, 2))
        # output = unfold(x)
        # print(output)
        # tensor([[ 1.,  2.,  3.,  5.,  6.,  7.,  9., 10., 11.],
        # [ 2.,  3.,  4.,  6.,  7.,  8., 10., 11., 12.],
        # [ 5.,  6.,  7.,  9., 10., 11., 13., 14., 15.],
        # [ 6.,  7.,  8., 10., 11., 12., 14., 15., 16.]])
        # padding=self.radius 为输入图像加上一定的边界填充。填充的大小由 self.radius 决定，这样可以确保即使在图像的边缘也能够提取出区域。
        self.unfold = nn.Unfold(kernel_size=self.kernel_size, padding=self.radius)

        # local xy grid
        # torch.linspace(-self.radius, self.radius, self.kernel_size) 生成了一个从 -self.radius 到 self.radius 之间的等间距数值序列。
        # 这个序列的长度等于 self.kernel_size，表示局部区域中每个点的坐标。
        x = torch.linspace(-self.radius, self.radius, self.kernel_size)
        # (kernel_size*kernel_size) x 2 : (w,h)
        # torch.meshgrid([x, x]) 生成了一个二维网格，用来表示局部区域中每个点的 (x, y) 坐标。这是一个 self.kernel_size x self.kernel_size 的网格。
        # torch.stack(...).view(2, -1).t() 将这个网格转换成一个二维数组，其中每一行表示一个局部点的 (x, y) 坐标。
        # [:, [1, 0]] 是为了调整坐标的顺序，使得每一行的元素顺序为 (h, w)，即先是行坐标 (height)，然后是列坐标 (width)。
        # 表达式 [:, [1, 0]] 是在调整张量的维度顺序，具体来说：
        # : 表示选择所有行。
        # [1, 0] 表示选择第二列（索引 1）和第一列（索引 0），并按这个顺序排列。
        self.hw_grid = torch.stack(torch.meshgrid([x, x])).view(2, -1).t()[:, [1, 0]]

    def detect_keypoints(self, scores_map, normalized_coordinates=True):
        b, c, h, w = scores_map.shape
        # 阻止反向传播：当你调用 detach() 后，返回的张量将不再具有与原张量相同的梯度计算关系，意味着对这个张量的操作将不会被用于梯度计算和反向传播。
        scores_nograd = scores_map.detach()
        # nms_scores = simple_nms(scores_nograd, self.radius)
        nms_scores = simple_nms(scores_nograd, 2)

        # remove border
        # 假设 nms_scores 是一个 4D 的张量，形状为 (batch_size, channels, height, width)。
        # nms_scores[:, :, :self.radius + 1, :] = 0 的含义如下：
        # : 表示在该维度上选择所有元素。
        # :self.radius + 1 在第三个维度（height 维度）上选择从 0 到 self.radius 的所有行（即前 self.radius + 1 行）。
        # = 号将这些行上的所有值设置为 0。
        nms_scores[:, :, :self.radius + 1, :] = 0
        nms_scores[:, :, :, :self.radius + 1] = 0
        nms_scores[:, :, h - self.radius:, :] = 0
        nms_scores[:, :, :, w - self.radius:] = 0

        # detect keypoints without grad
        if self.top_k > 0:
            # view(b, -1) 将 nms_scores 重塑为形状为 (batch_size, height * width) 的 2D 张量。
            # torch.topk 函数返回给定维度上最大的 k 个元素及其索引。
            topk = torch.topk(nms_scores.view(b, -1), self.top_k)
            indices_keypoints = topk.indices  # B x top_k
        else:
            if self.scores_th > 0:
                # masks 是一个布尔张量，表示 nms_scores 中每个像素的分数是否大于阈值 scores_th。形状为 (batch_size, channels, height, width)。
                # masks 中 True 的位置代表那些被选中的关键点。
                masks = nms_scores > self.scores_th
                if masks.sum() == 0:
                    # th = scores_nograd.reshape(b, -1).mean(dim=1)
                    # 如果没有像素分数超过指定阈值，代码会计算每张图片分数的平均值作为新的动态阈值。
                    # scores_nograd.reshape(b, -1) 将每张图片的分数展平成形状为 (batch_size, height * width) 的 2D 张量。
                    # mean(dim=1) 计算每张图片的平均分数，结果形状为 (batch_size,)。
                    # 5. masks = nms_scores > th.reshape(b, 1, 1, 1)
                    # 使用新的动态阈值 th 生成新的 masks，形状调整为 (batch_size, 1, 1, 1) 以便与 nms_scores 进行广播操作。
                    th = scores_nograd.reshape(b, -1).mean(dim=1)  # th = self.scores_th
                    masks = nms_scores > th.reshape(b, 1, 1, 1)
            else:
                th = scores_nograd.reshape(b, -1).mean(dim=1)  # th = self.scores_th
                masks = nms_scores > th.reshape(b, 1, 1, 1)
            # 将 masks 张量重新调整为形状 (batch_size, height * width)，即将每张图片中的所有像素点展开为一个一维向量。
            # 这样做的目的是方便后续处理，例如在关键点检测中快速提取所有被选中的关键点的索引。
            # 假设原始 masks 的形状是 (batch_size, channels, height, width)，通常 channels = 1，所以有效的形状为 (batch_size, height, width)。调整后的形状为 (batch_size, height * width)。
            # batch_size 表示批次大小，每个图片独立处理。
            # height * width 是图片的所有像素点数，展平为一个向量。
            # 目的
            # 将 masks 张量展平成二维张量后，可以方便地在每张图片中以线性方式操作，例如使用 .nonzero() 找出哪些位置（像素）被标记为 True，即可能的关键点位置。
            # 示例
            # 假设 masks 的形状是 (2, 1, 4, 4)，即有 2 张图片，每张图片的大小为 4x4。使用 reshape 后，masks 的形状会变为 (2, 16)
            masks = masks.reshape(b, -1)
            # 初始化一个空列表 indices_keypoints，用于存储每张图片中检测到的关键点索引。
            # indices_keypoints 是一个列表，它将存储每个批次中每张图片的关键点索引。
            # 每个图片的关键点索引是一个一维张量，因此列表的长度为批次大小（B），列表中的每个元素可以包含任意数量的索引（关键点数量不固定，因此是 “(any size)”）。
            # 例如，在一个批次中，如果有两张图片且每张图片分别检测到了 5 和 8 个关键点，则 indices_keypoints 会是一个包含两个元素的列表，形如 [tensor([...]), tensor([...])]。
            indices_keypoints = []  # list, B x (any size)
            # (b, c, h, w) 变为 (b, h * w)
            scores_view = scores_nograd.reshape(b, -1)
            # zip(masks, scores_view) 将 masks 和 scores_view 的对应元素打包在一起。每次迭代时，mask 是一个形状为 (h * w) 的布尔张量，
            # scores 是一个形状为 (h * w) 的分数张量。每一对 mask 和 scores 都代表当前批次中某一张图片的掩码和得分。
            # 目的是遍历每一张图片的 mask 和 scores，根据掩码 mask 提取出可能是关键点的位置，然后进一步处理这些位置的得分。
            for mask, scores in zip(masks, scores_view):
                # mask = tensor([True, False, True, False, True, False, False, True, False])
                # 执行 mask.nonzero(as_tuple=False) 会得到：
                # tensor([[0],
                # [2],
                # [4],
                # [7]])
                # 通过 [:, 0]，我们可以得到：tensor([0, 2, 4, 7])
                indices = mask.nonzero(as_tuple=False)[:, 0]
                if len(indices) > self.n_limit:
                    # scores 是一个一维张量，形状为 (h * w,)，表示图像中每个像素的分数。
                    # indices 是之前从 mask 中提取出的索引，表示关键点的位置。
                    # scores[indices] 提取出这些索引对应的分数，结果保存在 kpts_sc 中。
                    kpts_sc = scores[indices]
                    # kpts_sc.sort(descending=True) 会按照分数从高到低排序，返回一个元组 (sorted_values, sorted_indices)。
                    # sorted_values 是排序后的分数。
                    # sorted_indices 是原始 kpts_sc 中每个元素的索引，按照从高到低排序。
                    sort_idx = kpts_sc.sort(descending=True)[1]
                    # 这行代码从排序后的索引中选择前 n_limit 个元素。
                    sel_idx = sort_idx[:self.n_limit]
                    indices = indices[sel_idx]
                indices_keypoints.append(indices)

        # detect soft keypoints with grad backpropagation
        patches = self.unfold(scores_map)  # B x (kernel**2) x (H*W)
        self.hw_grid = self.hw_grid.to(patches)  # to device
        keypoints = []
        scoredispersitys = []
        kptscores = []
        for b_idx in range(b):
            patch = patches[b_idx].t()  # (H*W) x (kernel**2)
            indices_kpt = indices_keypoints[b_idx]  # one dimension vector, say its size is M
            patch_scores = patch[indices_kpt]  # M x (kernel**2)

            # max is detached to prevent undesired backprop loops in the graph
            max_v = patch_scores.max(dim=1).values.detach()[:, None]
            x_exp = ((patch_scores - max_v) / self.temperature).exp()  # M * (kernel**2), in [0, 1]

            # \frac{ \sum{(i,j) \times \exp(x/T)} }{ \sum{\exp(x/T)} }
            xy_residual = x_exp @ self.hw_grid / x_exp.sum(dim=1)[:, None]  # Soft-argmax, Mx2

            hw_grid_dist2 = torch.norm((self.hw_grid[None, :, :] - xy_residual[:, None, :]) / self.radius,
                                       dim=-1) ** 2
            scoredispersity = (x_exp * hw_grid_dist2).sum(dim=1) / x_exp.sum(dim=1)

            # compute result keypoints
            keypoints_xy_nms = torch.stack([indices_kpt % w, indices_kpt // w], dim=1)  # Mx2
            keypoints_xy = keypoints_xy_nms + xy_residual
            if normalized_coordinates:
                keypoints_xy = keypoints_xy / keypoints_xy.new_tensor([w - 1, h - 1]) * 2 - 1  # (w,h) -> (-1~1,-1~1)

            kptscore = torch.nn.functional.grid_sample(scores_map[b_idx].unsqueeze(0), keypoints_xy.view(1, 1, -1, 2),
                                                       mode='bilinear', align_corners=True)[0, 0, 0, :]  # CxN

            keypoints.append(keypoints_xy)
            scoredispersitys.append(scoredispersity)
            kptscores.append(kptscore)

        return keypoints, scoredispersitys, kptscores

    def forward(self, scores_map, descriptor_map, normalized_coordinates=True):
        """
        :param scores_map:  Bx1xHxW
        :param descriptor_map: BxCxHxW
        :return: kpts: list[Nx2,...]; kptscores: list[N,....] normalised position: -1.0 ~ 1.0
        """
        keypoints, scoredispersitys, kptscores = self.detect_keypoints(scores_map,
                                                                       normalized_coordinates)

        descriptors = sample_descriptor(descriptor_map, keypoints)

        # keypoints: B M 2
        # descriptors: B M D
        # scoredispersitys:
        return keypoints, descriptors, kptscores, scoredispersitys
