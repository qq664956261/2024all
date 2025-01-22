#!/usr/bin/env python
#
# %BANNER_BEGIN%
# ---------------------------------------------------------------------
# %COPYRIGHT_BEGIN%
#
#  Magic Leap, Inc. ("COMPANY") CONFIDENTIAL
#
#  Unpublished Copyright (c) 2018
#  Magic Leap, Inc., All Rights Reserved.
#
# NOTICE:  All information contained herein is, and remains the property
# of COMPANY. The intellectual and technical concepts contained herein
# are proprietary to COMPANY and may be covered by U.S. and Foreign
# Patents, patents in process, and are protected by trade secret or
# copyright law.  Dissemination of this information or reproduction of
# this material is strictly forbidden unless prior written permission is
# obtained from COMPANY.  Access to the source code contained herein is
# hereby forbidden to anyone except current COMPANY employees, managers
# or contractors who have executed Confidentiality and Non-disclosure
# agreements explicitly covering such access.
#
# The copyright notice above does not evidence any actual or intended
# publication or disclosure  of  this source code, which includes
# information that is confidential and/or proprietary, and is a trade
# secret, of  COMPANY.   ANY REPRODUCTION, MODIFICATION, DISTRIBUTION,
# PUBLIC  PERFORMANCE, OR PUBLIC DISPLAY OF OR THROUGH USE  OF THIS
# SOURCE CODE  WITHOUT THE EXPRESS WRITTEN CONSENT OF COMPANY IS
# STRICTLY PROHIBITED, AND IN VIOLATION OF APPLICABLE LAWS AND
# INTERNATIONAL TREATIES.  THE RECEIPT OR POSSESSION OF  THIS SOURCE
# CODE AND/OR RELATED INFORMATION DOES NOT CONVEY OR IMPLY ANY RIGHTS
# TO REPRODUCE, DISCLOSE OR DISTRIBUTE ITS CONTENTS, OR TO MANUFACTURE,
# USE, OR SELL ANYTHING THAT IT  MAY DESCRIBE, IN WHOLE OR IN PART.
#
# %COPYRIGHT_END%
# ----------------------------------------------------------------------
# %AUTHORS_BEGIN%
#
#  Originating Authors: Daniel DeTone (ddetone)
#                       Tomasz Malisiewicz (tmalisiewicz)
#
# %AUTHORS_END%
# --------------------------------------------------------------------*/
# %BANNER_END%


import argparse
import glob
import numpy as np
import os
import time

import cv2
import torch

# Stub to warn about opencv version.
if int(cv2.__version__[0]) < 3: # pragma: no cover
  print('Warning: OpenCV 3 is not installed')

# Jet colormap for visualization.
myjet = np.array([[0.        , 0.        , 0.5       ],
                  [0.        , 0.        , 0.99910873],
                  [0.        , 0.37843137, 1.        ],
                  [0.        , 0.83333333, 1.        ],
                  [0.30044276, 1.        , 0.66729918],
                  [0.66729918, 1.        , 0.30044276],
                  [1.        , 0.90123457, 0.        ],
                  [1.        , 0.48002905, 0.        ],
                  [0.99910873, 0.07334786, 0.        ],
                  [0.5       , 0.        , 0.        ]])

class SuperPointNet(torch.nn.Module):
  """ Pytorch definition of SuperPoint Network. """
  def __init__(self):
    super(SuperPointNet, self).__init__()
    #定义一个 ReLU 激活函数（inplace=True 表示对输入进行原地修改以节省内存）。
    self.relu = torch.nn.ReLU(inplace=True)
    # 定义一个最大池化层（kernel_size=2, stride=2），用于将输入的特征图尺寸减半。
    self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2)
    # 定义通道数的变量（c1, c2, c3, c4, c5, d1），用于卷积层的输出通道数和描述符的维度
    c1, c2, c3, c4, c5, d1 = 64, 64, 128, 128, 256, 256#64 表示卷积核的数量，即该层会使用 64 个不同的卷积核，每个卷积核会生成一个特征图。最终，卷积层的输出特征图会有 64 个通道，这 64 个特征图提供了输入数据的 64 个不同的视角或特征表示。
    # Shared Encoder.
    # 定义共享编码器部分的卷积层。
    # 卷积层的参数为输入通道数、输出通道数、卷积核大小、步幅、填充
#     1. 卷积操作的基本原理
# 卷积操作是深度学习中常用的技术，尤其是在图像处理和计算机视觉任务中。卷积层通过卷积核（或滤波器）对输入数据进行局部感受域的操作，从而提取出特征。

# 卷积核（滤波器）
# 卷积核 是一个小的权重矩阵（例如 3x3 或 5x5），它会在输入数据上滑动并计算加权和。这个过程称为卷积操作。
# 卷积核 通过与输入数据的局部区域进行点积计算，从而产生一个输出值。这个输出值构成了特征图中的一个像素点。
# 2. 参数解释
# torch.nn.Conv2d 的主要参数包括：

# in_channels: 输入数据的通道数。例如，对于灰度图像，这个值为 1；对于 RGB 图像，这个值为 3。
# out_channels: 卷积层输出的通道数（即卷积核的数量）。每个卷积核会生成一个特征图，因此 out_channels 决定了输出特征图的数量。
# kernel_size: 卷积核的大小。例如，kernel_size=3 表示卷积核的大小是 3x3。
# stride: 步幅，卷积核在输入数据上滑动的步长。步幅越大，输出特征图的尺寸越小。
# padding: 填充，向输入数据的边缘添加额外的像素，以控制输出特征图的尺寸。常用的填充方式有 "same"（保持尺寸不变）和 "valid"（无填充）。
# 3. 如何提升维度
# 卷积操作本质上不会直接提升数据的维度，但通过 out_channels 参数可以实现“维度的提升”：

# 增加通道数：卷积层的输出通道数（out_channels）可以设置为大于输入通道数（in_channels）。这就意味着，每个卷积核会生成一个新的特征图，输出特征图的数量等于卷积核的数量。
# 特征提取：卷积操作的目的是提取特征，不仅仅是提取局部特征，还可以在多个卷积层的堆叠中捕捉更复杂的特征。这种层叠结构可以提升特征的维度和复杂度。
# 4. 维度变化的计算
# 对于一个输入特征图，其尺寸由以下公式计算：

# 输出特征图的宽度和高度可以通过公式计算：

# Output Size=（Input Size−Kernel Size+2×Padding）/Stride+1

# Input Size 是输入特征图的尺寸（宽度或高度）。
# Kernel Size 是卷积核的大小。
# Padding 是填充的大小。
# Stride 是步幅。
# 通过调整这些参数，可以控制输出特征图的尺寸，从而实现特定的需求。

# 5. 总结
# torch.nn.Conv2d 通过卷积操作提取特征，并通过设置多个卷积核来增加输出特征图的数量（即“维度”）。它不会直接改变输入数据的空间尺寸，但可以通过设置输出通道数来提升特征的复杂性和表示能力。
    self.conv1a = torch.nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)# 从 1 个通道（灰度图像）到 c1 个通道。
    self.conv1b = torch.nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)#为什么不是 64 × 64卷积核的作用: 卷积核在每个输入通道上产生一个二维的特征图，并且每个卷积核在所有输入通道上产生的特征图都被叠加到输出特征图中。
    self.conv2a = torch.nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
    self.conv2b = torch.nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
    self.conv3a = torch.nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
    self.conv3b = torch.nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
    self.conv4a = torch.nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
    self.conv4b = torch.nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)
    # Detector Head.
    self.convPa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
    self.convPb = torch.nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)#这种配置常用于网络的最后几层，用于将特征图的通道数调整为所需的输出维度，而不对特征图的空间尺寸产生影响
    # Descriptor Head.
    self.convDa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
    self.convDb = torch.nn.Conv2d(c5, d1, kernel_size=1, stride=1, padding=0)
  # 网络在前向传播过程中的具体计算步骤。该函数的主要任务是将输入图像通过网络进行处理，生成点特征图 (semi) 和描述符特征图 (desc)。
  def forward(self, x):
    """ Forward pass that jointly computes unprocessed point and descriptor
    tensors.
    Input
      x: Image pytorch tensor shaped N x 1 x H x W.
    Output
      semi: Output point pytorch tensor shaped N x 65 x H/8 x W/8.
      desc: Output descriptor pytorch tensor shaped N x 256 x H/8 x W/8.
    """
   # Shared Encoder.
    # 对输入 x 进行卷积操作，并通过 ReLU 激活函数
    x = self.relu(self.conv1a(x))  # (N x 1 x H x W) -> (N x c1 x H x W)
    x = self.relu(self.conv1b(x))  # (N x c1 x H x W) -> (N x c1 x H x W)
    x = self.pool(x)               # 通过最大池化层减小空间维度: (N x c1 x H x W) -> (N x c1 x H/2 x W/2)
    x = self.relu(self.conv2a(x))  # (N x c1 x H/2 x W/2) -> (N x c2 x H/2 x W/2)
    x = self.relu(self.conv2b(x))  # (N x c2 x H/2 x W/2) -> (N x c2 x H/2 x W/2)
    x = self.pool(x)               # 通过最大池化层减小空间维度: (N x c2 x H/2 x W/2) -> (N x c2 x H/4 x W/4)
    x = self.relu(self.conv3a(x))  # (N x c2 x H/4 x W/4) -> (N x c3 x H/4 x W/4)
    x = self.relu(self.conv3b(x))  # (N x c3 x H/4 x W/4) -> (N x c3 x H/4 x W/4)
    x = self.pool(x)               # 通过最大池化层减小空间维度: (N x c3 x H/4 x W/4) -> (N x c3 x H/8 x W/8)
    x = self.relu(self.conv4a(x))  # (N x c3 x H/8 x W/8) -> (N x c4 x H/8 x W/8)
    x = self.relu(self.conv4b(x))  # (N x c4 x H/8 x W/8) -> (N x c4 x H/8 x W/8)
    
    # Detector Head.
    # 使用卷积层生成点特征图
    cPa = self.relu(self.convPa(x))  # (N x c4 x H/8 x W/8) -> (N x c5 x H/8 x W/8)
    semi = self.convPb(cPa)          # (N x c5 x H/8 x W/8) -> (N x 65 x H/8 x W/8)
    
    # Descriptor Head.
    # 使用卷积层生成描述符特征图
    cDa = self.relu(self.convDa(x))  # (N x c4 x H/8 x W/8) -> (N x c5 x H/8 x W/8)
    desc = self.convDb(cDa)          # (N x c5 x H/8 x W/8) -> (N x d1 x H/8 x W/8)
    
    # 归一化描述符
    dn = torch.norm(desc, p=2, dim=1)  # 计算描述符的 L2 范数: (N x d1 x H/8 x W/8) -> (N x 1 x H/8 x W/8)
    desc = desc.div(torch.unsqueeze(dn, 1))  # 将描述符除以其范数进行归一化: (N x d1 x H/8 x W/8)
    
    return semi, desc  # 返回点特征图和描述符特征图

"""封装PyTorch网络以帮助进行图像的预处理和后处理的类。"""
class SuperPointFrontend(object):
  """ Wrapper around pytorch net to help with pre and post image processing. """
  def __init__(self, weights_path, nms_dist, conf_thresh, nn_thresh,
               cuda=False):
    """
    初始化SuperPointFrontend对象。

    参数:
      weights_path (str): 网络权重文件的路径。
      nms_dist (int): 非极大值抑制（NMS）的距离阈值。
      conf_thresh (float): 置信度阈值。
      nn_thresh (float): 用于描述子匹配的L2距离阈值。
      cuda (bool): 是否使用CUDA（GPU），默认为False。
    """
    self.name = 'SuperPoint'
    self.cuda = cuda
    self.nms_dist = nms_dist
    self.conf_thresh = conf_thresh
    self.nn_thresh = nn_thresh # L2 descriptor distance for good match.
    self.cell = 8 # Size of each output cell. Keep this fixed.
    self.border_remove = 4 # Remove points this close to the border.

    # Load the network in inference mode.
    self.net = SuperPointNet()
    if cuda:
      # Train on GPU, deploy on GPU.
      self.net.load_state_dict(torch.load(weights_path))
      self.net = self.net.cuda()
    else:
      # Train on GPU, deploy on CPU.
      self.net.load_state_dict(torch.load(weights_path,
                               map_location=lambda storage, loc: storage))
    self.net.eval()# 以推理模式加载网络。
  #实现了快速近似非极大值抑制（Non-Maximum Suppression, NMS），用于去除低置信度的角点，并只保留置信度高的角点。
  def nms_fast(self, in_corners, H, W, dist_thresh):
    """
    运行快速近似非极大值抑制，处理形状为 3xN 的 numpy 角点数组：
      3xN [x_i,y_i,conf_i]^T

    算法摘要：创建一个大小为 HxW 的网格。将每个角点位置赋值为 1，其余位置为 0。
    遍历所有值为 1 的位置，将它们转换为 -1 或 0。通过将附近的值设为 0 来抑制点。

    网格值图例：
    -1 : 保留的点。
     0 : 空的或被抑制的点。
     1 : 待处理的点（转换为保留或抑制）。

    注意：NMS 首先将点四舍五入为整数，因此 NMS 距离可能不完全是 dist_thresh。它还假定点在图像边界内。

    输入:
      in_corners - 3xN 的 numpy 数组，包含角点 [x_i, y_i, confidence_i]^T。
      H - 图像高度。
      W - 图像宽度。
      dist_thresh - 抑制距离，测量为无穷范数距离。
    返回:
      nmsed_corners - 3xN 的 numpy 矩阵，包含保留的角点。
      nmsed_inds - N 长度的 numpy 向量，包含保留的角点索引。
    """
     # 初始化网格和索引矩阵。
    grid = np.zeros((H, W)).astype(int) # Track NMS data.
    inds = np.zeros((H, W)).astype(int) # Store indices of points.
    # Sort by confidence and round to nearest int.
    # in_corners[2, :]：
    # in_corners 是一个形状为 3xN 的 numpy 数组，表示 N 个角点，每个角点包含 x 坐标、y 坐标和置信度。
    # in_corners[2, :] 选择角点数组的第三行，即每个角点的置信度值，得到一个长度为 N 的数组。
    # -in_corners[2, :]：
    # 负号 - 用于将置信度值取反，因为 np.argsort 默认是升序排列。取反后，置信度高的点会变成负值较小的数，这样在升序排列后，置信度高的点会排在前面。
    # np.argsort(-in_corners[2, :])：
    # np.argsort 函数返回的是排序后的索引。也就是说，它返回的是一个数组，其中的每个值是原数组中某个元素在排序后的位置。
    # 由于置信度值取了负号，所以 np.argsort 返回的索引数组会按置信度从高到低排序。
    inds1 = np.argsort(-in_corners[2,:])
    # in_corners 是一个形状为 3xN 的 numpy 数组，其中每一列表示一个角点，包含 x 坐标、y 坐标和置信度值。
    # inds1 是根据角点的置信度从高到低排序后的索引数组。
    # in_corners[:, inds1] 利用这些排序后的索引，重新排列原始角点数组，使其按置信度从高到低排列。
    corners = in_corners[:,inds1]
    # corners 是一个形状为 3xN 的 numpy 数组，其中每一列表示一个角点，包含 x 坐标、y 坐标和置信度值。
    # corners[:2, :] 提取出 corners 的前两行，即 x 坐标和 y 坐标。
    # .round() 对提取出的坐标进行四舍五入，以便得到整数坐标。
    # .astype(int) 将四舍五入后的坐标转换为整数类型。
    rcorners = corners[:2,:].round().astype(int) # Rounded corners.
    # Check for edge case of 0 or 1 corners.
    # 情况 1：没有角点（rcorners.shape[1] == 0）
    # rcorners.shape[1] 表示角点的数量，即 rcorners 数组的列数。
    # 如果角点数量为零（rcorners.shape[1] == 0），则没有角点需要处理。
    # np.zeros((3, 0)).astype(int) 创建一个形状为 (3, 0) 的空数组，并转换为整数类型。
    # np.zeros(0).astype(int) 创建一个长度为 0 的空数组，并转换为整数类型。
    # 返回这两个空数组，表示没有角点和对应的索引。
    # 情况 2：只有一个角点（rcorners.shape[1] == 1）
    # 如果角点数量为一（rcorners.shape[1] == 1），则只有一个角点需要处理。
    # np.vstack((rcorners, in_corners[2])) 将 rcorners（包含 x 和 y 坐标）与 in_corners 的置信度值（第三行）进行垂直堆叠。
    # reshape(3, 1) 将堆叠后的数组形状调整为 (3, 1)，表示一个角点的 x 坐标、y 坐标和置信度值。
    # np.zeros((1)).astype(int) 创建一个长度为 1 的数组，并转换为整数类型，用于表示这个角点的索引。
    # 返回包含角点信息的数组和对应的索引。
    if rcorners.shape[1] == 0:
      return np.zeros((3,0)).astype(int), np.zeros(0).astype(int)
    if rcorners.shape[1] == 1:
      out = np.vstack((rcorners, in_corners[2])).reshape(3,1)
      return out, np.zeros((1)).astype(int)
    # Initialize the grid.
    # enumerate(rcorners.T) 对转置后的 rcorners 数组进行迭代。
    # rcorners 是一个形状为 (2, N) 的数组，表示 N 个角点的 x 和 y 坐标。
    # rcorners.T 是其转置，形状为 (N, 2)，表示每一行是一个角点的 (x, y) 坐标。
    # enumerate 提供角点的索引 i 和对应的坐标 rc。
    # grid[rcorners[1, i], rcorners[0, i]] = 1 将每个角点的位置在 grid 中标记为 1。
    # rcorners[1, i] 是第 i 个角点的 y 坐标。
    # rcorners[0, i] 是第 i 个角点的 x 坐标。
    # 将 grid 中对应的位置设置为 1，表示该位置有一个角点。
    # inds[rcorners[1, i], rcorners[0, i]] = i 在 inds 数组中存储角点的索引。
    # inds 数组与 grid 数组的大小相同。
    # 将 inds 中对应的位置设置为 i，表示该位置的角点在原始角点数组中的索引。
    for i, rc in enumerate(rcorners.T):
      grid[rcorners[1,i], rcorners[0,i]] = 1
      inds[rcorners[1,i], rcorners[0,i]] = i
    # Pad the border of the grid, so that we can NMS points near the border.
    # 在 grid 数组的边缘添加 padding（填充），以处理角点位于图像边缘时的情况
    pad = dist_thresh
    grid = np.pad(grid, ((pad,pad), (pad,pad)), mode='constant')
    # Iterate through points, highest to lowest conf, suppress neighborhood.
    # count = 0：初始化计数器 count，用于记录处理过的角点数量。
    # for i, rc in enumerate(rcorners.T)：
    # 遍历 rcorners 数组的每一列，rcorners 是角点的坐标。
    # rcorners.T 是 rcorners 数组的转置，使得每一列表示一个角点的 [x, y] 坐标。
    # i 是角点的索引，rc 是角点的坐标 [x, y]。
    # pt = (rc[0] + pad, rc[1] + pad)：
    # 由于 grid 已经被填充了 pad 个像素的边界，这里将角点坐标调整到填充后的 grid 中的位置。
    # rc[0] 和 rc[1] 是角点的 x 和 y 坐标，pad 是填充的大小。
    # if grid[pt[1], pt[0]] == 1：
    # 检查 grid 中 pt 位置的值是否为 1。值为 1 表示该位置的角点尚未被抑制。
    # grid[pt[1] - pad:pt[1] + pad + 1, pt[0] - pad:pt[0] + pad + 1] = 0：
    # 对于角点 pt，将其邻域区域（根据 dist_thresh 确定的大小）设置为 0，表示该区域的角点已经被抑制。
    # pt[1] - pad 和 pt[0] - pad 是邻域区域的左上角坐标，pt[1] + pad + 1 和 pt[0] + pad + 1 是右下角坐标。
    # grid[pt[1], pt[0]] = -1：
    # 将角点 pt 的位置在 grid 中设置为 -1，表示该角点被保留。
    # count += 1：
    # 增加计数器 count，记录处理过的角点数量。
    count = 0
    for i, rc in enumerate(rcorners.T):
      # Account for top and left padding.
      pt = (rc[0]+pad, rc[1]+pad)
      if grid[pt[1], pt[0]] == 1: # If not yet suppressed.
        grid[pt[1]-pad:pt[1]+pad+1, pt[0]-pad:pt[0]+pad+1] = 0
        grid[pt[1], pt[0]] = -1
        count += 1
    # Get all surviving -1's and return sorted array of remaining corners.
    # np.where(grid == -1) 找到 grid 中所有值为 -1 的位置。这些位置对应于经过非极大值抑制后的角点。
    # keepy 和 keepx 分别是这些位置的 y 和 x 坐标。
    keepy, keepx = np.where(grid==-1)
    # 由于在 grid 上应用了 pad，我们需要将这些位置坐标从填充后的 grid 位置映射回原始图像的坐标。
    # keepy - pad 和 keepx - pad 将坐标还原到没有填充的原始 grid 上的位置。
    keepy, keepx = keepy - pad, keepx - pad
    # inds 数组存储了每个角点在原始输入中的索引。
    # inds_keep 是 keepy 和 keepx 中角点的索引，这些角点是在 NMS 后保留下来的。
    inds_keep = inds[keepy, keepx]
    # corners 包含了所有的角点信息，inds_keep 是经过 NMS 保留的角点的索引。
    # out 是保留下来的角点的信息，形状为 3 x N，其中 N 是经过 NMS 后保留的角点数量。
    out = corners[:, inds_keep]
    # out[-1, :] 是 out 的最后一行，表示这些保留角点的置信度值。
    # values 存储了这些保留角点的置信度。
    values = out[-1, :]
    # 对 values 进行排序，np.argsort(-values) 返回 values 从高到低的排序索引。
    # inds2 是按置信度从高到低排列的角点索引。
    inds2 = np.argsort(-values)
    out = out[:, inds2]
    out_inds = inds1[inds_keep[inds2]]
    return out, out_inds
  


  #用于处理输入图像，提取角点和描述符，并生成热力图。  
  def run(self, img):
    """ Process a numpy image to extract points and descriptors.
    Input
      img - HxW numpy float32 input image in range [0,1].
    Output
      corners - 3xN numpy array with corners [x_i, y_i, confidence_i]^T.
      desc - 256xN numpy array of corresponding unit normalized descriptors.
      heatmap - HxW numpy heatmap in range [0,1] of point confidences.
      """
    assert img.ndim == 2, 'Image must be grayscale.'
    assert img.dtype == np.float32, 'Image must be float32.'
    H, W = img.shape[0], img.shape[1]
    inp = img.copy()
    inp = (inp.reshape(1, H, W))
    inp = torch.from_numpy(inp)
    #用于封装一个张量（tensor），使其能够支持自动微分。尽管从 PyTorch 1.0 起，Variable 和 Tensor 的功能已经合并，通常直接使用 Tensor 即可，但在一些旧的代码中可能仍然会看到 Variable 的使用
    inp = torch.autograd.Variable(inp).view(1, 1, H, W)
    if self.cuda:
      inp = inp.cuda()
    # Forward pass of network.
    # 调用网络的前向传播
    outs = self.net.forward(inp)
    # outs[0] 是特征点置信度图（semi），形状为 [1, 65, H/8, W/8]。
    # outs[1] 是描述符图（coarse_desc），形状为 [1, 256, H/8, W/8]。
    # 在 conv1a 和 conv1b 之后进行池化操作，将特征图尺寸减半。
    # 在 conv2a 和 conv2b 之后进行池化操作，再次将特征图尺寸减半。
    # 在 conv3a 和 conv3b 之后进行池化操作，再次将特征图尺寸减半。
    # 最后在 conv4a 和 conv4b 之后没有进一步池化，但已经经过了三次池化，每次池化都将特征图的尺寸减半。
    semi, coarse_desc = outs[0], outs[1]
    # Convert pytorch -> numpy.
    semi = semi.data.cpu().numpy().squeeze()
    # --- Process points.
    # 将网络的原始输出转换为概率分布,这里使用了 softmax 函数将输出转换为概率
    dense = np.exp(semi) # Softmax.
    # 这一步是对 dense 进行归一化处理，使得每个位置的概率和为 1。
    dense = dense / (np.sum(dense, axis=0)+.00001) # Should sum to 1.
    # Remove dustbin.
    # 输出的 semi 通常包含了多个通道，每个通道表示某个可能的特征点类型或某个位置是否为特征点。最后一个通道通常被用作背景或者垃圾桶类，实际上在关键点检测中是没有实际意义的，主要用来填充。
    nodust = dense[:-1, :, :]
    # Reshape to get full resolution heatmap.
    # 由于特征图的每个单元大小为 self.cell x self.cell（这里是 8x8 像素），因此：
    # 特征图的高度 Hc 是原始图像高度 H 除以每个单元的大小 self.cell。
    # 特征图的宽度 Wc 是原始图像宽度 W 除以每个单元的大小 self.cell。
    # 这样计算出来的 Hc 和 Wc 就是网络输出特征图的分辨率，它们反映了每个特征点在原始图像中的位置。
    Hc = int(H / self.cell)
    Wc = int(W / self.cell)
    # nodust 是一个三维数组，其原始形状是 (num_classes, Hc, Wc)，其中 num_classes 是特征图中的类别数（不包括最后的“垃圾桶”类别）。
    # transpose 方法将其维度调整为 (Hc, Wc, num_classes)
    nodust = nodust.transpose(1, 2, 0)
    # 将 nodust 重塑为一个四维数组 heatmap，其形状是 (Hc, Wc, self.cell, self.cell)。
    # 这一步是将特征图的每个单元（cell）从一个平面（2D）扩展为一个小的 2D 区域，这个区域对应于实际图像中的 8x8 像素块。
    heatmap = np.reshape(nodust, [Hc, Wc, self.cell, self.cell])
    # transpose 操作将 heatmap 的维度从 (Hc, Wc, self.cell, self.cell) 变为 (Hc, self.cell, Wc, self.cell)。这一步是为了将每个单元的区域进行正确的排列，方便下一步的重塑操作。
    heatmap = np.transpose(heatmap, [0, 2, 1, 3])
    # 将 heatmap 重塑为一个二维数组，其形状是 (Hc*self.cell, Wc*self.cell)，即 (H, W)。这将特征图的分辨率恢复到接近原始图像的分辨率。
    heatmap = np.reshape(heatmap, [Hc*self.cell, Wc*self.cell])
    # np.where(heatmap >= self.conf_thresh) 返回 heatmap 中所有值大于等于置信度阈值 self.conf_thresh 的位置坐标 xs 和 ys
    xs, ys = np.where(heatmap >= self.conf_thresh) # Confidence threshold.
    # 如果没有找到任何高于阈值的点，它会返回空的特征点数组和 None
    if len(xs) == 0:
      return np.zeros((3, 0)), None, None
    # pts 是一个形状为 3xN 的零数组，其中 N 是满足置信度阈值的特征点的数量。
    # pts[0, :] = ys 将 ys 坐标填入 pts 的第一行。
    # pts[1, :] = xs 将 xs 坐标填入 pts 的第二行。
    # pts[2, :] = heatmap[xs, ys] 将对应的置信度值填入 pts 的第三行。
    pts = np.zeros((3, len(xs))) # Populate point data sized 3xN.
    pts[0, :] = ys
    pts[1, :] = xs
    pts[2, :] = heatmap[xs, ys]
    # 调用 self.nms_fast 方法对 pts 应用非极大值抑制，去除冗余的特征点。
    pts, _ = self.nms_fast(pts, H, W, dist_thresh=self.nms_dist) # Apply NMS.
    #  生成一个索引数组，根据 pts 第三行（置信度）对特征点进行排序。
    inds = np.argsort(pts[2,:])
    # 使用倒序索引数组对 pts 进行排序，确保特征点按置信度从高到低排列。
    pts = pts[:,inds[::-1]] # Sort by confidence.
    # Remove points along border.
    bord = self.border_remove
    # pts[0, :] 是特征点的横坐标（列）。
    # pts[0, :] < bord 检查特征点是否在左边界内 bord 像素范围内。
    # pts[0, :] >= (W-bord) 检查特征点是否在右边界内 bord 像素范围内。
    # np.logical_or 将这两个条件结合起来，得到一个布尔数组 toremoveW，表示哪些特征点在横向上需要移除。
    toremoveW = np.logical_or(pts[0, :] < bord, pts[0, :] >= (W-bord))
    # pts[1, :] 是特征点的纵坐标（行）。
    # pts[1, :] < bord 检查特征点是否在上边界内 bord 像素范围内。
    # pts[1, :] >= (H-bord) 检查特征点是否在下边界内 bord 像素范围内。
    # np.logical_or 将这两个条件结合起来，得到一个布尔数组 toremoveH，表示哪些特征点在纵向上需要移除。
    toremoveH = np.logical_or(pts[1, :] < bord, pts[1, :] >= (H-bord))
    # np.logical_or(toremoveW, toremoveH) 将横向和纵向的移除条件结合起来，得到一个最终的布尔数组 toremove，表示哪些特征点需要移除。
    toremove = np.logical_or(toremoveW, toremoveH)
    # ~toremove 是 toremove 的反转，表示哪些特征点不需要移除。
    # pts[:, ~toremove] 将这些特征点保留下来，从而移除了靠近边界的特征点。
    pts = pts[:, ~toremove]
    # --- Process descriptor.
    # D 是描述子的维度，coarse_desc 的形状为 (N, D, H/8, W/8)，其中 N 是批次大小，D 是描述子的维度。
    D = coarse_desc.shape[1]
    # 如果没有特征点，则返回一个形状为 (D, 0) 的空描述子数组。
    if pts.shape[1] == 0:
      desc = np.zeros((D, 0))
    else:
      # Interpolate into descriptor map using 2D point locations.
      # pts[:2, :] 提取特征点的 x 和 y 坐标，转换为 PyTorch 张量 samp_pts。
      samp_pts = torch.from_numpy(pts[:2, :].copy())
      # 将特征点的坐标归一化到 [-1, 1] 范围，方便后续使用 grid_sample 进行插值。
      # 归一化公式： (samp_pts / (size / 2)) - 1，将坐标从图像空间映射到 [-1, 1] 空间。
      samp_pts[0, :] = (samp_pts[0, :] / (float(W)/2.)) - 1.
      samp_pts[1, :] = (samp_pts[1, :] / (float(H)/2.)) - 1.
      # 将张量的形状从 (2, N) 转换为 (N, 2) 并确保内存连续。
      samp_pts = samp_pts.transpose(0, 1).contiguous()
      # samp_pts.view(1, 1, -1, 2) 将张量的形状调整为 (1, 1, N, 2)，这是 grid_sample 函数所需的输入形状。
      samp_pts = samp_pts.view(1, 1, -1, 2)
      samp_pts = samp_pts.float()
      if self.cuda:
        samp_pts = samp_pts.cuda()
        # 在 coarse_desc 上进行插值，得到每个特征点对应的描述子。
      desc = torch.nn.functional.grid_sample(coarse_desc, samp_pts)
      # ：将插值后的描述子从 PyTorch 张量转换为 NumPy 数组，并调整形状为 (D, N)。
      desc = desc.data.cpu().numpy().reshape(D, -1)
      # 对描述子进行归一化，使其每列的 L2 范数为 1。
      desc /= np.linalg.norm(desc, axis=0)[np.newaxis, :]
    return pts, desc, heatmap


class PointTracker(object):
  """ Class to manage a fixed memory of points and descriptors that enables
  sparse optical flow point tracking.

  Internally, the tracker stores a 'tracks' matrix sized M x (2+L), of M
  tracks with maximum length L, where each row corresponds to:
  row_m = [track_id_m, avg_desc_score_m, point_id_0_m, ..., point_id_L-1_m].
  """

  def __init__(self, max_length, nn_thresh):
    if max_length < 2:
      raise ValueError('max_length must be greater than or equal to 2.')
    self.maxl = max_length#表示轨迹的最大长度。
    self.nn_thresh = nn_thresh#表示最近邻匹配的阈值，用于决定两个描述符是否匹配
    #self.all_pts：这是一个列表，存储每帧图像中的点。每个元素是一个 2x0 的零数组，表示没有点（初始化为空）。
    self.all_pts = []
    for n in range(self.maxl):
      self.all_pts.append(np.zeros((2, 0)))
    #用于存储最后一帧的描述符
    self.last_desc = None
    # self.tracks：这是一个大小为 0 x (self.maxl + 2) 的数组，用于存储轨迹信息。每行表示一条轨迹，其中包括 track_id、avg_desc_score 以及每个时间步的点 ID。
    self.tracks = np.zeros((0, self.maxl+2))
    self.track_count = 0
    self.max_score = 9999
  # 实现了双向最近邻匹配，即在两个描述符集合之间找到双向最近邻匹配。这意味着从描述符 A 到描述符 B 的最近邻匹配必须等于从 B 到 A 的最近邻匹配
  def nn_match_two_way(self, desc1, desc2, nn_thresh):
    """
    Performs two-way nearest neighbor matching of two sets of descriptors, such
    that the NN match from descriptor A->B must equal the NN match from B->A.

    Inputs:
      desc1 - NxM numpy matrix of N corresponding M-dimensional descriptors.
      desc2 - NxM numpy matrix of N corresponding M-dimensional descriptors.
      nn_thresh - Optional descriptor distance below which is a good match.

    Returns:
      matches - 3xL numpy array, of L matches, where L <= N and each column i is
                a match of two descriptors, d_i in image 1 and d_j' in image 2:
                [d_i index, d_j' index, match_score]^T
    """
    # 确保两个描述符矩阵具有相同的行数
    assert desc1.shape[0] == desc2.shape[0]
    # 如果任意一个描述符矩阵为空，则返回空匹配
    if desc1.shape[1] == 0 or desc2.shape[1] == 0:
      return np.zeros((3, 0))
    if nn_thresh < 0.0:
      raise ValueError('\'nn_thresh\' should be non-negative')
    # Compute L2 distance. Easy since vectors are unit normalized.
    # 计算 L2 距离矩阵

    dmat = np.dot(desc1.T, desc2)
    dmat = np.sqrt(2-2*np.clip(dmat, -1, 1))
    # Get NN indices and scores.
    # 对于每个描述符，找到距离最近的描述符的索引和对应的距离分数。
        # dmat 是一个二维矩阵，存储了描述符之间的距离。假设 dmat 的形状为 (N, M)，其中 N 是 desc1 中描述符的数量，M 是 desc2 中描述符的数量。
    # np.argmin(dmat, axis=1) 返回一个长度为 N 的数组 idx，其中每个元素表示在 dmat 矩阵中，每行最小值所在的列索引。也就是说，
    # 对于 desc1 中的每个描述符，找到与 desc2 中最相似的描述符的索引。
    # 对于 dmat 矩阵的每一行，找到该行中的最小值的列索引。
    idx = np.argmin(dmat, axis=1)
    # idx = np.argmin(dmat, axis=1) 计算如下：
    # 第1行最小值是 0.1，位于索引 0 处
    # 第2行最小值是 0.2，位于索引 1 处
    # 第3行最小值是 0.2，位于索引 2 处
    # 因此 idx = [0, 1, 2]
    # scores = dmat[np.arange(dmat.shape[0]), idx] 计算如下：
    # 从 dmat 中提取 dmat[0, 0]，即 0.1
    # 从 dmat 中提取 dmat[1, 1]，即 0.2
    # 从 dmat 中提取 dmat[2, 2]，即 0.2
    # 因此 scores = [0.1, 0.2, 0.2]
    scores = dmat[np.arange(dmat.shape[0]), idx]
    # Threshold the NN matches.
    # scores[0] = 0.1，小于 0.4，所以 keep[0] = True
    # scores[1] = 0.3，小于 0.4，所以 keep[1] = True
    # scores[2] = 0.5，大于 0.4，所以 keep[2] = False
    # scores[3] = 0.2，小于 0.4，所以 keep[3] = True
    keep = scores < nn_thresh
    # Check if nearest neighbor goes both directions and keep those.
    # 作用: 对于 dmat 矩阵的每一列，找到该列中的最小值的行索引。
    idx2 = np.argmin(dmat, axis=0)
    # 最近邻索引
    # idx 是每一行在距离矩阵 dmat 中最近邻的列索引。具体来说，对于矩阵中的每一行 
    # i，idx[i] 是该行的最近邻所在的列索引。
    # idx2 是每一列在距离矩阵 dmat 中最近邻的行索引。对于矩阵中的每一列 
    # j，idx2[j] 是该列的最近邻所在的行索引。
    # 互相匹配的检查：
    # idx2[idx] 通过使用 idx 中的值来重新索引 idx2。这一步的目的是获取 idx 指定的列的最近邻行索引。
    # np.arange(len(idx)) 创建了一个从 0 到 len(idx)-1 的索引数组。
    # 比较：
    # np.arange(len(idx)) == idx2[idx] 创建了一个布尔数组，其中每个元素为 True，如果 np.arange(len(idx)) 中的索引等于 idx2[idx] 中的值。这一步用来检查最近邻关系是否是互相匹配的。
    keep_bi = np.arange(len(idx)) == idx2[idx]
    keep = np.logical_and(keep, keep_bi)
    idx = idx[keep]
    scores = scores[keep]
    # Get the surviving point indices.
    m_idx1 = np.arange(desc1.shape[1])[keep]
    m_idx2 = idx
    # Populate the final 3xN match data structure.
    matches = np.zeros((3, int(keep.sum())))
    matches[0, :] = m_idx1
    matches[1, :] = m_idx2
    matches[2, :] = scores
    return matches

  def get_offsets(self):
    """ Iterate through list of points and accumulate an offset value. Used to
    index the global point IDs into the list of points.

    Returns
      offsets - N length array with integer offset locations.
    """
    # Compute id offsets.
    offsets = []
    offsets.append(0)
    for i in range(len(self.all_pts)-1): # Skip last camera size, not needed.
      offsets.append(self.all_pts[i].shape[1])
    offsets = np.array(offsets)
    offsets = np.cumsum(offsets)
    return offsets

  def update(self, pts, desc):
    """ Add a new set of point and descriptor observations to the tracker.

    Inputs
      pts - 3xN numpy array of 2D point observations.
      desc - DxN numpy array of corresponding D dimensional descriptors.
    """
    if pts is None or desc is None:
      print('PointTracker: Warning, no points were added to tracker.')
      return
    assert pts.shape[1] == desc.shape[1]
    # Initialize last_desc.
    if self.last_desc is None:
      self.last_desc = np.zeros((desc.shape[0], 0))
    # Remove oldest points, store its size to update ids later.
    remove_size = self.all_pts[0].shape[1]
    self.all_pts.pop(0)
    self.all_pts.append(pts)
    # Remove oldest point in track.
    self.tracks = np.delete(self.tracks, 2, axis=1)
    # Update track offsets.
    for i in range(2, self.tracks.shape[1]):
      self.tracks[:, i] -= remove_size
    self.tracks[:, 2:][self.tracks[:, 2:] < -1] = -1
    offsets = self.get_offsets()
    # Add a new -1 column.
    self.tracks = np.hstack((self.tracks, -1*np.ones((self.tracks.shape[0], 1))))
    # Try to append to existing tracks.
    matched = np.zeros((pts.shape[1])).astype(bool)
    matches = self.nn_match_two_way(self.last_desc, desc, self.nn_thresh)
    for match in matches.T:
      # Add a new point to it's matched track.
      id1 = int(match[0]) + offsets[-2]
      id2 = int(match[1]) + offsets[-1]
      found = np.argwhere(self.tracks[:, -2] == id1)
      if found.shape[0] > 0:
        matched[int(match[1])] = True
        row = int(found)
        self.tracks[row, -1] = id2
        if self.tracks[row, 1] == self.max_score:
          # Initialize track score.
          self.tracks[row, 1] = match[2]
        else:
          # Update track score with running average.
          # NOTE(dd): this running average can contain scores from old matches
          #           not contained in last max_length track points.
          track_len = (self.tracks[row, 2:] != -1).sum() - 1.
          frac = 1. / float(track_len)
          self.tracks[row, 1] = (1.-frac)*self.tracks[row, 1] + frac*match[2]
    # Add unmatched tracks.
    new_ids = np.arange(pts.shape[1]) + offsets[-1]
    new_ids = new_ids[~matched]
    new_tracks = -1*np.ones((new_ids.shape[0], self.maxl + 2))
    new_tracks[:, -1] = new_ids
    new_num = new_ids.shape[0]
    new_trackids = self.track_count + np.arange(new_num)
    new_tracks[:, 0] = new_trackids
    new_tracks[:, 1] = self.max_score*np.ones(new_ids.shape[0])
    self.tracks = np.vstack((self.tracks, new_tracks))
    self.track_count += new_num # Update the track count.
    # Remove empty tracks.
    keep_rows = np.any(self.tracks[:, 2:] >= 0, axis=1)
    self.tracks = self.tracks[keep_rows, :]
    # Store the last descriptors.
    self.last_desc = desc.copy()
    return

  def get_tracks(self, min_length):
    """ Retrieve point tracks of a given minimum length.
    Input
      min_length - integer >= 1 with minimum track length
    Output
      returned_tracks - M x (2+L) sized matrix storing track indices, where
        M is the number of tracks and L is the maximum track length.
    """
    if min_length < 1:
      raise ValueError('\'min_length\' too small.')
    valid = np.ones((self.tracks.shape[0])).astype(bool)
    good_len = np.sum(self.tracks[:, 2:] != -1, axis=1) >= min_length
    # Remove tracks which do not have an observation in most recent frame.
    not_headless = (self.tracks[:, -1] != -1)
    keepers = np.logical_and.reduce((valid, good_len, not_headless))
    returned_tracks = self.tracks[keepers, :].copy()
    return returned_tracks

  def draw_tracks(self, out, tracks):
    """ Visualize tracks all overlayed on a single image.
    Inputs
      out - numpy uint8 image sized HxWx3 upon which tracks are overlayed.
      tracks - M x (2+L) sized matrix storing track info.
    """
    # Store the number of points per camera.
    pts_mem = self.all_pts
    N = len(pts_mem) # Number of cameras/images.
    # Get offset ids needed to reference into pts_mem.
    offsets = self.get_offsets()
    # Width of track and point circles to be drawn.
    stroke = 1
    # Iterate through each track and draw it.
    for track in tracks:
      clr = myjet[int(np.clip(np.floor(track[1]*10), 0, 9)), :]*255
      for i in range(N-1):
        if track[i+2] == -1 or track[i+3] == -1:
          continue
        offset1 = offsets[i]
        offset2 = offsets[i+1]
        idx1 = int(track[i+2]-offset1)
        idx2 = int(track[i+3]-offset2)
        pt1 = pts_mem[i][:2, idx1]
        pt2 = pts_mem[i+1][:2, idx2]
        p1 = (int(round(pt1[0])), int(round(pt1[1])))
        p2 = (int(round(pt2[0])), int(round(pt2[1])))
        cv2.line(out, p1, p2, clr, thickness=stroke, lineType=16)
        # Draw end points of each track.
        if i == N-2:
          clr2 = (255, 0, 0)
          cv2.circle(out, p2, stroke, clr2, -1, lineType=16)

class VideoStreamer(object):
  """ Class to help process image streams. Three types of possible inputs:"
    1.) USB Webcam.
    2.) A directory of images (files in directory matching 'img_glob').
    3.) A video file, such as an .mp4 or .avi file.
  """
  def __init__(self, basedir, camid, height, width, skip, img_glob):
    self.cap = []
    self.camera = False
    self.video_file = False
    self.listing = []
    self.sizer = [height, width]
    self.i = 0
    self.skip = skip
    self.maxlen = 1000000
    # If the "basedir" string is the word camera, then use a webcam.
    if basedir == "camera/" or basedir == "camera":
      print('==> Processing Webcam Input.')
      self.cap = cv2.VideoCapture(camid)
      self.listing = range(0, self.maxlen)
      self.camera = True
    else:
      # Try to open as a video.
      self.cap = cv2.VideoCapture(basedir)
      lastbit = basedir[-4:len(basedir)]
      if (type(self.cap) == list or not self.cap.isOpened()) and (lastbit == '.mp4'):
        raise IOError('Cannot open movie file')
      elif type(self.cap) != list and self.cap.isOpened() and (lastbit != '.txt'):
        print('==> Processing Video Input.')
        num_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.listing = range(0, num_frames)
        self.listing = self.listing[::self.skip]
        self.camera = True
        self.video_file = True
        self.maxlen = len(self.listing)
      else:
        print('==> Processing Image Directory Input.')
        search = os.path.join(basedir, img_glob)
        self.listing = glob.glob(search)
        self.listing.sort()
        self.listing = self.listing[::self.skip]
        self.maxlen = len(self.listing)
        if self.maxlen == 0:
          raise IOError('No images were found (maybe bad \'--img_glob\' parameter?)')

  def read_image(self, impath, img_size):
    """ Read image as grayscale and resize to img_size.
    Inputs
      impath: Path to input image.
      img_size: (W, H) tuple specifying resize size.
    Returns
      grayim: float32 numpy array sized H x W with values in range [0, 1].
    """
    grayim = cv2.imread(impath, 0)
    if grayim is None:
      raise Exception('Error reading image %s' % impath)
    # Image is resized via opencv.
    interp = cv2.INTER_AREA
    grayim = cv2.resize(grayim, (img_size[1], img_size[0]), interpolation=interp)
    grayim = (grayim.astype('float32') / 255.)
    return grayim

  def next_frame(self):
    """ Return the next frame, and increment internal counter.
    Returns
       image: Next H x W image.
       status: True or False depending whether image was loaded.
    """
    if self.i == self.maxlen:
      return (None, False)
    if self.camera:
      ret, input_image = self.cap.read()
      if ret is False:
        print('VideoStreamer: Cannot get image from camera (maybe bad --camid?)')
        return (None, False)
      if self.video_file:
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.listing[self.i])
      input_image = cv2.resize(input_image, (self.sizer[1], self.sizer[0]),
                               interpolation=cv2.INTER_AREA)
      input_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2GRAY)
      input_image = input_image.astype('float')/255.0
    else:
      image_file = self.listing[self.i]
      input_image = self.read_image(image_file, self.sizer)
    # Increment internal counter.
    self.i = self.i + 1
    input_image = input_image.astype('float32')
    return (input_image, True)


if __name__ == '__main__':

  # Parse command line arguments.
  parser = argparse.ArgumentParser(description='PyTorch SuperPoint Demo.')
  parser.add_argument('input', type=str, default='',
      help='Image directory or movie file or "camera" (for webcam).')
  parser.add_argument('--weights_path', type=str, default='superpoint_v1.pth',
      help='Path to pretrained weights file (default: superpoint_v1.pth).')
  parser.add_argument('--img_glob', type=str, default='*.png',
      help='Glob match if directory of images is specified (default: \'*.png\').')
  parser.add_argument('--skip', type=int, default=1,
      help='Images to skip if input is movie or directory (default: 1).')
  parser.add_argument('--show_extra', action='store_true',
      help='Show extra debug outputs (default: False).')
  parser.add_argument('--H', type=int, default=120,
      help='Input image height (default: 120).')
  parser.add_argument('--W', type=int, default=160,
      help='Input image width (default:160).')
  parser.add_argument('--display_scale', type=int, default=2,
      help='Factor to scale output visualization (default: 2).')
  parser.add_argument('--min_length', type=int, default=2,
      help='Minimum length of point tracks (default: 2).')
  parser.add_argument('--max_length', type=int, default=5,
      help='Maximum length of point tracks (default: 5).')
  parser.add_argument('--nms_dist', type=int, default=4,
      help='Non Maximum Suppression (NMS) distance (default: 4).')
  parser.add_argument('--conf_thresh', type=float, default=0.015,
      help='Detector confidence threshold (default: 0.015).')
  parser.add_argument('--nn_thresh', type=float, default=0.7,
      help='Descriptor matching threshold (default: 0.7).')
  parser.add_argument('--camid', type=int, default=0,
      help='OpenCV webcam video capture ID, usually 0 or 1 (default: 0).')
  parser.add_argument('--waitkey', type=int, default=1,
      help='OpenCV waitkey time in ms (default: 1).')
  parser.add_argument('--cuda', action='store_true',
      help='Use cuda GPU to speed up network processing speed (default: False)')
  parser.add_argument('--no_display', action='store_true',
      help='Do not display images to screen. Useful if running remotely (default: False).')
  parser.add_argument('--write', action='store_true',
      help='Save output frames to a directory (default: False)')
  parser.add_argument('--write_dir', type=str, default='tracker_outputs/',
      help='Directory where to write output frames (default: tracker_outputs/).')
  opt = parser.parse_args()
  print(opt)

  # This class helps load input images from different sources.
  vs = VideoStreamer(opt.input, opt.camid, opt.H, opt.W, opt.skip, opt.img_glob)

  print('==> Loading pre-trained network.')
  # This class runs the SuperPoint network and processes its outputs.
  fe = SuperPointFrontend(weights_path=opt.weights_path,
                          nms_dist=opt.nms_dist,
                          conf_thresh=opt.conf_thresh,
                          nn_thresh=opt.nn_thresh,
                          cuda=opt.cuda)
  print('==> Successfully loaded pre-trained network.')

  # This class helps merge consecutive point matches into tracks.
  tracker = PointTracker(opt.max_length, nn_thresh=fe.nn_thresh)

  # Create a window to display the demo.
  if not opt.no_display:
    win = 'SuperPoint Tracker'
    cv2.namedWindow(win)
  else:
    print('Skipping visualization, will not show a GUI.')

  # Font parameters for visualizaton.
  font = cv2.FONT_HERSHEY_DUPLEX
  font_clr = (255, 255, 255)
  font_pt = (4, 12)
  font_sc = 0.4

  # Create output directory if desired.
  if opt.write:
    print('==> Will write outputs to %s' % opt.write_dir)
    if not os.path.exists(opt.write_dir):
      os.makedirs(opt.write_dir)

  print('==> Running Demo.')
  while True:

    start = time.time()

    # Get a new image.
    img, status = vs.next_frame()
    if status is False:
      break

    # Get points and descriptors.
    start1 = time.time()
    pts, desc, heatmap = fe.run(img)
    end1 = time.time()

    # Add points and descriptors to the tracker.
    tracker.update(pts, desc)

    # Get tracks for points which were match successfully across all frames.
    tracks = tracker.get_tracks(opt.min_length)

    # Primary output - Show point tracks overlayed on top of input image.
    out1 = (np.dstack((img, img, img)) * 255.).astype('uint8')
    tracks[:, 1] /= float(fe.nn_thresh) # Normalize track scores to [0,1].
    tracker.draw_tracks(out1, tracks)
    if opt.show_extra:
      cv2.putText(out1, 'Point Tracks', font_pt, font, font_sc, font_clr, lineType=16)

    # Extra output -- Show current point detections.
    out2 = (np.dstack((img, img, img)) * 255.).astype('uint8')
    for pt in pts.T:
      pt1 = (int(round(pt[0])), int(round(pt[1])))
      cv2.circle(out2, pt1, 1, (0, 255, 0), -1, lineType=16)
    cv2.putText(out2, 'Raw Point Detections', font_pt, font, font_sc, font_clr, lineType=16)

    # Extra output -- Show the point confidence heatmap.
    if heatmap is not None:
      min_conf = 0.001
      heatmap[heatmap < min_conf] = min_conf
      heatmap = -np.log(heatmap)
      heatmap = (heatmap - heatmap.min()) / (heatmap.max() - heatmap.min() + .00001)
      out3 = myjet[np.round(np.clip(heatmap*10, 0, 9)).astype('int'), :]
      out3 = (out3*255).astype('uint8')
    else:
      out3 = np.zeros_like(out2)
    cv2.putText(out3, 'Raw Point Confidences', font_pt, font, font_sc, font_clr, lineType=16)

    # Resize final output.
    if opt.show_extra:
      out = np.hstack((out1, out2, out3))
      out = cv2.resize(out, (3*opt.display_scale*opt.W, opt.display_scale*opt.H))
    else:
      out = cv2.resize(out1, (opt.display_scale*opt.W, opt.display_scale*opt.H))

    # Display visualization image to screen.
    if not opt.no_display:
      cv2.imshow(win, out)
      key = cv2.waitKey(opt.waitkey) & 0xFF
      if key == ord('q'):
        print('Quitting, \'q\' pressed.')
        break

    # Optionally write images to disk.
    if opt.write:
      out_file = os.path.join(opt.write_dir, 'frame_%05d.png' % vs.i)
      print('Writing image to %s' % out_file)
      cv2.imwrite(out_file, out)

    end = time.time()
    net_t = (1./ float(end1 - start))
    total_t = (1./ float(end - start))
    if opt.show_extra:
      print('Processed image %d (net+post_process: %.2f FPS, total: %.2f FPS).'\
            % (vs.i, net_t, total_t))

  # Close any remaining windows.
  cv2.destroyAllWindows()

  print('==> Finshed Demo.')
