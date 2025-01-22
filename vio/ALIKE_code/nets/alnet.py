import math
import torch
from torch import nn
# from pytorch_lightning.core.lightning import LightningModule
from pytorch_lightning import LightningModule
from torchvision.models import resnet
from typing import Optional, Callable

# BaseNet 类的构造函数（__init__ 方法）在初始化时调用了父类 LightningModule 的构造函数，即 super().__init__()。
# 当前 BaseNet 类没有定义其他属性或方法，它只是一个占位符，用于被其他子类继承和扩展。
# 在实际应用中，你通常会在 BaseNet 或继承自 BaseNet 的子类中定义模型的网络层（如卷积层、全连接层）和前向传播方法 (forward 方法)。
# 这个设计模式的好处在于，BaseNet 可以被其他子类复用，提供一个统一的接口或通用的功能，而具体的模型实现可以在子类中完成。
class BaseNet(LightningModule):
    def __init__(self, ):
        super().__init__()

    def forward(self, *args, **kwargs):
        raise NotImplementedError()

# 继承自 nn.Module
# ConvBlock 的作用
# ConvBlock 是一个典型的卷积神经网络块，它将卷积、归一化、和激活函数结合在一起，以形成一个基本的功能模块。这个模块可以在更大的神经网络结构中重复使用，比如 ResNet 等模型中
# 如果 in_channels = 64，out_channels = 128，那么这个卷积层将有 128 个 3x3 的卷积核。
# 每个卷积核的深度是 64，即每个卷积核的形状是 64x3x3。这个卷积层最终会输出 128 个通道的特征图。
# 初始化 (__init__)：
# in_channels: 输入张量的通道数。
# out_channels: 输出张量的通道数。
# gate: 激活函数，默认为 nn.ReLU。
# norm_layer: 归一化层，默认为 nn.BatchNorm2d。
# conv1 和 conv2 使用了 torchvision.models.resnet 中的 conv3x3 函数，这是一个 3x3 的卷积层。
# bn1 和 bn2 分别是第一层和第二层的批归一化层。
# 前向传播 (forward)
# 输入数据先通过第一个卷积层和归一化层，接着通过激活函数。
# 然后通过第二个卷积层和归一化层，接着通过激活函数。
# 最后返回处理后的张量。
class ConvBlock(nn.Module):
    def __init__(self, in_channels, out_channels,
                 gate: Optional[Callable[..., nn.Module]] = None,
                 norm_layer: Optional[Callable[..., nn.Module]] = None):
        super().__init__()
        if gate is None:
            self.gate = nn.ReLU(inplace=True)
        else:
            self.gate = gate
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        self.conv1 = resnet.conv3x3(in_channels, out_channels)
        self.bn1 = norm_layer(out_channels)
        self.conv2 = resnet.conv3x3(out_channels, out_channels)
        self.bn2 = norm_layer(out_channels)

    def forward(self, x):
        x = self.gate(self.bn1(self.conv1(x)))  # B x in_channels x H x W
        x = self.gate(self.bn2(self.conv2(x)))  # B x out_channels x H x W
        return x


# copied from torchvision\models\resnet.py#27->BasicBlock
# 这个 ResBlock 是一个标准的 ResNet 残差块，包含了卷积、归一化、激活和残差连接。通过这些操作，残差块能够有效地捕获图像的高级特征，同时保持梯度的顺畅传递。
# ResBlock 是一个典型的残差块（Residual Block），它是 ResNet（残差网络）中最基本的构建模块。
# 在这个实现中，ResBlock 由两个卷积层组成，并且通过跳跃连接（skip connection）来实现残差学习
# 1. 跳跃连接（Skip Connection）
# 定义: 跳跃连接是指在神经网络中，将某一层的输入直接连接到后面的某一层，而不仅仅是经过一系列的层次（如卷积层、激活函数等）后再传递。这种连接方式通常是通过加法操作（out += identity）来实现的。
# 目的: 跳跃连接的主要目的是解决深层神经网络中的梯度消失问题。通过直接传递输入的信息，它允许梯度更容易地反向传播到前面的层，改善了模型的训练效果。
# 实现: 在 ResNet 中，跳跃连接通常是将输入 x（或经过下采样后的 x，称为 identity）直接与卷积层的输出 out 相加。
# 2. 下采样（Downsampling）
# 定义: 下采样是指通过某种方法（如卷积、池化等）减少特征图的空间维度（即宽度和高度），通常会增加感受野，同时减小计算量。
# 目的: 下采样的主要目的是减小特征图的尺寸，以便在深层网络中逐步提取更高级别的特征，同时保持计算效率。它通常用于卷积神经网络中的池化层或者步幅大于1的卷积层。
# 实现: 在 ResNet 的 ResBlock 中，下采样可以通过一个步幅为2的卷积层或池化层来实现，用于减少特征图的尺寸。
# 为什么跳跃连接和下采样经常一起出现？
# 在残差块中，跳跃连接将输入 x 直接与输出 out 相加。如果输入和输出的特征图维度不一致（例如宽高不同或通道数不同），则不能直接相加。这时就需要使用下采样来调整 x 的维度，使其与 out 的维度一致。

# 下采样: 如果在残差块的卷积层中使用了步幅为2的卷积（导致特征图尺寸减半），那么就需要在跳跃连接中对 x 进行下采样，以匹配输出的尺寸。这通常是通过一个卷积层来实现的（称为 downsample 层）。
# 跳跃连接: 然后，将下采样后的 x 直接与经过卷积层处理后的输出 out 相加。这一操作实现了跳跃连接，并确保两者的尺寸一致。
class ResBlock(nn.Module):
    expansion: int = 1

    def __init__(
            self,
            inplanes: int,
            planes: int,
            stride: int = 1,
            downsample: Optional[nn.Module] = None,
            groups: int = 1,
            base_width: int = 64,
            dilation: int = 1,
            gate: Optional[Callable[..., nn.Module]] = None,
            norm_layer: Optional[Callable[..., nn.Module]] = None
    ) -> None:
        super(ResBlock, self).__init__()
        if gate is None:
            self.gate = nn.ReLU(inplace=True)
        else:
            self.gate = gate
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        if groups != 1 or base_width != 64:
            raise ValueError('ResBlock only supports groups=1 and base_width=64')
        if dilation > 1:
            raise NotImplementedError("Dilation > 1 not supported in ResBlock")
        # Both self.conv1 and self.downsample layers downsample the input when stride != 1
        self.conv1 = resnet.conv3x3(inplanes, planes, stride)
        self.bn1 = norm_layer(planes)
        self.conv2 = resnet.conv3x3(planes, planes)
        self.bn2 = norm_layer(planes)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        identity = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.gate(out)

        out = self.conv2(out)
        out = self.bn2(out)

        if self.downsample is not None:
            identity = self.downsample(x)

        out += identity
        out = self.gate(out)

        return out

# 这个 PositionEncodingSine 类是一个基于正弦和余弦函数的位置信息编码（Positional Encoding）模块，
# 通常用于给卷积神经网络（CNN）或 transformer 网络添加位置信息，以便模型能够更好地理解输入数据中的空间位置信息。
# 关键点:
# 位置编码的目的：添加关于每个元素在空间网格中位置的信息，帮助模型利用空间关系。
# 拼接：通过将位置编码与原始输入拼接在一起，模型可以在一个张量中同时获取原始信息和位置编码信息。
# 灵活性：位置编码中使用正弦函数可以让模型轻松捕捉不同尺度的关系。
# 这种方法在处理图像等空间数据时非常有效，可以帮助模型更好地理解空间关系。
class PositionEncodingSine(LightningModule):
    def __init__(self):
        super().__init__()

        # pe = torch.zeros((8, *max_shape))
        # y_position = torch.ones(max_shape).cumsum(0).float().unsqueeze(0)
        # x_position = torch.ones(max_shape).cumsum(1).float().unsqueeze(0)
        # pe[0, :, :] = x_position
        # pe[1, :, :] = y_position
        # pe[2, :, :] = torch.sin(x_position)
        # pe[3, :, :] = torch.cos(y_position)
        # pe[4, :, :] = torch.sin(x_position * 0.5)
        # pe[5, :, :] = torch.cos(y_position * 0.5)
        # pe[6, :, :] = torch.cos(x_position * 0.25)
        # pe[7, :, :] = torch.cos(y_position * 0.25)
        #
        # self.register_buffer('pe', pe.unsqueeze(0), persistent=False)  # [1, C, H, W]
    # x 是输入的特征图，形状为 [N, C, H, W]，其中 N 是批次大小，C 是通道数，H 和 W 分别是高度和宽度。
    def forward(self, x):
        """
        Args:
            x: [N, C, H, W]
        """
        _, _, h, w = x.shape
        shape = (h, w)
        # 计算位置编码:
        # pe 被初始化为一个形状为 [8, H, W] 的全零张量，用来存储不同类型的位置编码。
        # y_position 和 x_position 是表示垂直和水平方向上位置的累积和的张量，并通过除以 h 和 w 进行归一化。
        pe = torch.zeros(8, *shape, device=x.device)
        y_position = torch.ones(shape, device=x.device).cumsum(0).float().unsqueeze(0) / h
        x_position = torch.ones(shape, device=x.device).cumsum(1).float().unsqueeze(0) / w
        # 创建位置编码:
        # 计算不同类型的位置编码并存储在 pe 张量中。这些编码包括简单的位置索引（x_position 和 y_position），以及带有不同频率的正弦编码。
        pe[0, :, :] = x_position
        pe[1, :, :] = y_position
        pe[2, :, :] = torch.sin(x_position * 3.14 * 2)
        pe[3, :, :] = torch.cos(y_position * 3.14 * 2)
        # pe[4, :, :] = torch.sin(x_position * 3.14 * w / 3)
        # pe[5, :, :] = torch.cos(y_position * 3.14 * h / 3)
        # pe[6, :, :] = torch.cos(x_position * 3.14 * w / 5)
        # pe[7, :, :] = torch.cos(y_position * 3.14 * h / 5)
        pe[4, :, :] = torch.sin(x_position * 3.14 * 8)
        pe[5, :, :] = torch.cos(y_position * 3.14 * 8)
        pe[6, :, :] = torch.cos(x_position * 3.14 * 32)
        pe[7, :, :] = torch.cos(y_position * 3.14 * 32)
        # 将位置编码张量 pe 裁剪到与输入张量 x 相同的大小，并在通道维度上将它们拼接在一起。结果是一个包含了位置信息和原始信息的张量。
        return torch.cat([x, pe[None, :, :x.size(2), :x.size(3)]], dim=1)

# 用于特征提取和检测。它继承自 BaseNet 并实现了一些特定的特征编码、特征聚合以及检测头部功能。
# 参数说明:
# c1, c2, c3, c4, dim: 这些参数定义了网络的通道数和特征维度。
# agg_mode: 特征聚合模式，可以是 'sum', 'cat', 或 'fpn'。
# single_head: 是否使用单一检测头。
# pe: 是否使用位置编码。
# 三种特征聚合模式：sum、cat 和 fpn。它们的主要区别在于如何将来自不同层次的特征融合在一起，以生成最终的特征表示。以下是对这三种模式的详细解释：
# 1. sum 模式
# 方法： 在 sum 模式下，网络将不同层次的特征通过卷积层处理后，直接进行加和。
# 操作流程：
# 每个特征层（x1, x2, x3, x4）分别通过卷积层进行降维（即 conv1, conv2, conv3, conv4）。
# 然后，将这些特征通过上采样层（upsample2, upsample8, upsample32）调整到相同的空间大小。
# 最后，将这些调整后的特征逐步相加，得到最终的聚合特征。
# 优点：
# 计算简单，速度快。
# 通过加和操作，可以有效融合不同尺度的信息。
# 缺点：
# 由于直接相加，可能会导致信息丢失，特别是对于尺度较大的特征图。
# 2. cat 模式
# 方法： 在 cat 模式下，网络将不同层次的特征在通道维度上进行拼接。
# 操作流程：
# 每个特征层（x1, x2, x3, x4）分别通过卷积层进行降维，但每个特征层降维后的通道数是最终特征通道数的四分之一。
# 然后，将这些特征通过上采样层调整到相同的空间大小。
# 最后，将这些调整后的特征在通道维度上拼接，形成最终的聚合特征。
# 优点：
# 保留了更多的细节信息，因为不同层次的特征直接拼接在一起。
# 更适合需要保留多尺度信息的任务。
# 缺点：
# 由于拼接操作，会增加特征的通道数，从而导致计算量增加。
# 需要处理不同尺度特征间的差异。
# 3. fpn 模式
# 方法： 在 fpn 模式下，网络采用类似特征金字塔网络（Feature Pyramid Network, FPN）的方式逐层融合特征。
# 操作流程：
# 从最深层的特征（x4）开始，通过卷积层处理后，上采样到上一层的空间大小，并与上一层的特征（x3）相加。
# 继续这一过程，逐层向上直到最浅层的特征（x1）。
# 每一层的特征都是通过前一层的特征上采样并与当前层的特征相加来得到的。
# 优点：
# 能够有效融合不同尺度的信息，同时保留高层次特征的语义信息和低层次特征的细节信息。
# 在对象检测、语义分割等任务中表现良好。
# 缺点：
# 相对复杂，计算量可能比 sum 模式大一些。
# 需要仔细调节每一层特征的融合方式。
# 总结
# sum 模式：简单直接，适合需要快速计算的场景。
# cat 模式：保留更多细节，适合需要多尺度信息的任务，但计算量较大。
# fpn 模式：综合了高层次语义信息和低层次细节信息，适合对象检测和分割任务。
class ALNet(BaseNet):
    def __init__(self, c1: int = 32, c2: int = 64, c3: int = 128, c4: int = 128, dim: int = 128,
                 agg_mode: str = 'cat',  # sum, cat, fpn
                 single_head: bool = True,
                 pe: bool = False,
                 ):
        super().__init__()

        self.gate = nn.ReLU(inplace=True)

        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        self.pool4 = nn.MaxPool2d(kernel_size=4, stride=4)

        self.pe = pe

        # ================================== feature encoder
        if self.pe:
            self.position_encoding = PositionEncodingSine()
            # 表示位置编码被应用。位置编码会为输入图像增加8个通道（通常对应于不同的正弦和余弦函数），因此，ConvBlock 的输入通道数将从 3（RGB 图像的通道数）增加到 3 + 8 = 11。
            self.block1 = ConvBlock(3 + 8, c1, self.gate, nn.BatchNorm2d)
        else:
            self.block1 = ConvBlock(3, c1, self.gate, nn.BatchNorm2d)
        # downsample=nn.Conv2d(c1, c2, 1):
        # 当输入和输出的通道数不同（例如从 c1 到 c2），downsample 用于调整输入的尺寸以匹配输出的尺寸。
        # 这些残差块的设计使得网络能够逐步减少空间分辨率，同时增加特征图的通道数，提取出更高层次、更抽象的特征。在每个块中，
        # 使用 1x1 卷积进行下采样，确保输入的通道数和输出的通道数匹配

        # 这里的 "下采样" 其实是指在通道数不匹配时通过 1x1 卷积对特征图的通道数进行调整，
        # 而不是对特征图的空间尺寸进行下采样（即减少宽度和高度）。这种操作有时也被称为“通道下采样”或“通道匹配”。
        # 使用 1x1 卷积进行通道下采样时，会使用 c2 数量的卷积核来进行调整。每个卷积核的尺寸是 1x1，并且这些卷积核共同作用，将输入特征图的通道数从 c1 调整为 c2。
        self.block2 = ResBlock(inplanes=c1, planes=c2, stride=1,
                               downsample=nn.Conv2d(c1, c2, 1),
                               gate=self.gate,
                               norm_layer=nn.BatchNorm2d)
        self.block3 = ResBlock(inplanes=c2, planes=c3, stride=1,
                               downsample=nn.Conv2d(c2, c3, 1),
                               gate=self.gate,
                               norm_layer=nn.BatchNorm2d)
        self.block4 = ResBlock(inplanes=c3, planes=c4, stride=1,
                               downsample=nn.Conv2d(c3, c4, 1),
                               gate=self.gate,
                               norm_layer=nn.BatchNorm2d)

        # ================================== feature aggregation
        # sum 模式：
        # 在 sum 模式下，各个层的特征图会被逐层加和，形成最终的聚合特征图。
        # 具体实现中，每个层（block1, block2, block3, block4）的输出通道数先通过 1x1 卷积变换为 dim 个通道，然后经过上采样操作，使得它们的空间尺寸相同。
        # 最后，这些上采样后的特征图会逐层相加，得到一个总和的特征图。
        # fpn 模式：
        # fpn（Feature Pyramid Network）模式类似于 sum，但是聚合过程更加复杂。
        # 在这个模式下，先从最高层（block4）开始，逐层上采样并加和到前一层的特征图上。每一层的输出先经过 1x1 卷积变换为 dim 个通道，然后与上采样后的上一层特征图相加。
        # 这个过程形成一个自底向上的特征金字塔，从而产生更加丰富的特征表示。
        # cat 模式：
        # 在 cat 模式下，各层的特征图会通过通道维度直接拼接（concatenate）在一起，而不是相加。
        # 为了保持拼接后的通道数与 dim 对应，每层的输出通道数会通过 1x1 卷积变换为 dim // 4 个通道。
        # 最终，将所有层的特征图在通道维度拼接在一起，得到聚合后的特征图。
        self.agg_mode = agg_mode
        if self.agg_mode == 'sum' or self.agg_mode == 'fpn':
            self.conv1 = resnet.conv1x1(c1, dim)
            self.conv2 = resnet.conv1x1(c2, dim)
            self.conv3 = resnet.conv1x1(c3, dim)
            self.conv4 = resnet.conv1x1(dim, dim)
        elif self.agg_mode == 'cat':
            self.conv1 = resnet.conv1x1(c1, dim // 4)
            self.conv2 = resnet.conv1x1(c2, dim // 4)
            self.conv3 = resnet.conv1x1(c3, dim // 4)
            self.conv4 = resnet.conv1x1(dim, dim // 4)
        else:
            raise ValueError(f"Unkown aggregation mode: '{self.agg_mode}', should be 'sum' or 'cat'!")
        # 定义了四个不同的上采样层（Upsample），它们用于将特征图的空间分辨率放大到不同的尺度。每个上采样层的作用是通过插值方法将特征图的尺寸扩大。
        # nn.Upsample 是一个用于上采样的 PyTorch 层，它可以将输入张量的空间尺寸（宽度和高度）增加到指定的倍数。可以通过设置 scale_factor 来控制放大的比例。
        # 参数说明
        # scale_factor: 这是一个倍数，指定了要放大特征图的尺度。例如，scale_factor=2 表示将特征图的宽度和高度都放大到原来的两倍。
        # mode: 插值模式，这里使用的是 'bilinear'，即双线性插值。它在两个方向上使用线性插值来计算新值，通常用于图像数据。
        # align_corners: 当设置为 True 时，插值计算将对齐角点，这在某些应用场景中可以提高插值质量。False 时不会对齐角点。
        # 各个上采样层的作用
        # self.upsample2:
        # 将特征图的尺寸放大 2 倍。用于将特征图从较小的尺度上采样到较大的尺度。
        # self.upsample4:
        # 将特征图的尺寸放大 4 倍。适用于更大的尺度放大。
        # self.upsample8:
        # 将特征图的尺寸放大 8 倍。适用于更大尺度的上采样。
        # self.upsample32:
        # 将特征图的尺寸放大 32 倍。适用于需要显著放大特征图的场景。
        self.upsample2 = nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        self.upsample4 = nn.Upsample(scale_factor=4, mode='bilinear', align_corners=True)
        self.upsample8 = nn.Upsample(scale_factor=8, mode='bilinear', align_corners=True)
        self.upsample32 = nn.Upsample(scale_factor=32, mode='bilinear', align_corners=True)

        # ================================== detector and descriptor head
        #     当 single_head 设置为 True 时，模型确实只有描述子头，特征点的检测和描述子提取是一起完成的。
        # 当 single_head 设置为 False 时，模型有两个头，convhead1 提供了额外的处理步骤，以帮助提高特征点检测和描述子提取的质量。
        # 因此，不使用 convhead1 时，模型的确只有描述子头，而特征点检测仍由 convhead2 的最后一个通道负责，只是没有额外的处理步骤。
        # 1x1 卷积的确只有一个像素的感受野，但它的卷积过程和其他卷积核是类似的。下面是解释1x1 卷积如何工作的详细步骤：
        # 1x1 卷积的工作原理
        # ，也就是说，这个卷积核的宽度和高度都是1，但它会遍历输入特征图的每一个通道。
        # 卷积过程：
        # 在执行1x1 卷积时，卷积核会在输入特征图的每个位置（像素）上滑动。由于卷积核的大小是 
        # 1×1，它只会在每个位置对单个像素进行运算。
        # 对于输入特征图的每个位置，1x1 卷积核会对所有输入通道进行加权求和，即：
        # 1x1 卷积的实际作用
        # 通道混合： 1x1 卷积能够对输入的每个像素位置的所有通道进行线性组合。这样，它可以将不同通道的信息重新组合，生成新的特征表示。
        # 维度变换： 它可以用于将输入特征图的通道数改变为输出特征图的通道数（例如，增加或减少通道数）。
        # 非线性映射： 在1x1 卷积之后通常会接一个非线性激活函数（如 ReLU），这使得网络能够在不改变空间维度的情况下增加非线性表达能力。

        # 举例说明
        # 如果输入特征图的大小是 64×64×128
        # 64×64×128，即高度和宽度分别为64像素，通道数为128。使用一个1x1卷积核，假设输出通道数为256，则：

        # 每个 1×1 的卷积核会对输入特征图中每个位置的128个通道进行线性组合，并生成一个新的输出通道。
        # 这样，输出特征图的大小变为 64 × 64 × 256
        # 64×64×256，即保持空间尺寸不变，但通道数增加。
        # 所以，尽管1x1卷积的感受野看起来很小，但它在通道维度上的操作使得它非常强大，特别是在深度神经网络中。
        self.single_head = single_head
        if not self.single_head:
            self.convhead1 = resnet.conv1x1(dim, dim)
        self.convhead2 = resnet.conv1x1(dim, dim + 1)

    def forward(self, image):
        # ================================== feature encoder
        # 如果 self.pe 为真，网络首先对输入图像进行位置编码 (PositionEncodingSine)。
        # 位置编码将位置信息（例如，像素的绝对位置）注入到输入图像中，使得网络能够识别图像中的空间位置信息。
        # 然后，经过 block1，它通常是一些卷积层，将输入的通道数变为 c1，同时保持空间尺寸不变。
        # 如果 self.pe 为假，则跳过位置编码，直接将图像输入到 block1
        if self.pe:
            x1 = self.position_encoding(image)
            x1 = self.block1(x1)  # B x c1 x H x W
        else:
            x1 = self.block1(image)  # B x c1 x H x W
        # x1 经过池化层 (pool2) 进行下采样，通常是最大池化，池化核的大小是 2×2，步长为2，所以空间尺寸减半。
        # 经过 block2 之后，通道数从 c1 增加到 c2，同时空间尺寸变为原来的 1/2。
        x2 = self.pool2(x1)
        x2 = self.block2(x2)  # B x c2 x H/2 x W/2
        # x2 再次经过池化层 (pool4) 进行更大步长的下采样，池化核的大小是 4×4，步长为4，空间尺寸再次减少。
        # 经过 block3 之后，通道数从 c2 增加到 c3，同时空间尺寸进一步变为原来的 1/8。
        x3 = self.pool4(x2)
        x3 = self.block3(x3)  # B x c3 x H/8 x W/8
        # x3 再次经过池化层进行下采样，空间尺寸进一步减少。
        # 最后，经过 block4，通道数从 c3 增加到 dim，空间尺寸变为原来的 1/32。
        x4 = self.pool4(x3)
        x4 = self.block4(x4)  # B x dim x H/32 x W/32

        # ================================== feature aggregation
        # 这个模式下，首先将每个尺度的特征图 (x1, x2, x3, x4) 都通过 1x1 卷积（conv1, conv2, conv3, conv4）将其通道数统一为 dim。
        # 然后，对较小尺度的特征图进行上采样，使其与最大尺度的特征图（x1，原始输入尺寸）具有相同的空间尺寸。
        # 最后，将所有尺度的特征图逐层相加（通过上采样后的 x2, x3, x4 累加到 x1）。
        # 效果：
        # 这种方式将不同尺度的特征融合成相同的尺寸，利用每个尺度的信息，同时保留了特征的全局性和局部性。
        if self.agg_mode == 'sum':
            x1234 = self.gate(self.conv1(x1))  # B x dim x H x W
            x2 = self.gate(self.conv2(x2))  # B x dim x H//2 x W//2
            x1234 = x1234 + self.upsample2(x2)
            x3 = self.gate(self.conv3(x3))  # B x dim x H//8 x W//8
            x1234 = x1234 + self.upsample8(x3)
            x4 = self.gate(self.conv4(x4))  # B x dim x H//32 x W//32
            x1234 = x1234 + self.upsample32(x4)
            # 这种方式类似于特征金字塔网络（FPN）的思想，从最小尺度（x4）开始，依次进行上采样，并与上一层（空间尺寸较大的特征图）进行逐元素相加。
            # 例如，x4 经过上采样后与 x3 进行相加，然后再与 x2 相加，最后与 x1 相加。
            # 效果：
            # 这种方式在融合过程中逐渐增加特征图的分辨率，从而可以更好地融合不同尺度的信息，适合用于检测任务中对不同尺度目标的捕捉。
        elif self.agg_mode == 'fpn':
            x1234 = self.gate(self.conv4(x4))  # B x dim x H//32 x W//32
            x1234 = self.upsample4(x1234)  # B x dim x H//8 x W//8
            x1234 = self.gate(self.conv3(x3) + x1234)  # B x dim x H//8 x W//8
            x1234 = self.upsample4(x1234)  # B x dim x H//2 x W//2
            x1234 = self.gate(self.conv2(x2) + x1234)  # B x dim x H//2 x W//2
            x1234 = self.upsample2(x1234)  # B x dim x H x W
            x1234 = self.gate(self.conv1(x1) + x1234)  # B x dim x H x W
            # 这种方式下，先将每个尺度的特征图通过 1x1 卷积压缩到相同的通道数（dim//4）。
            # 然后，将较小尺度的特征图进行上采样，使其空间尺寸与最大尺度的特征图相同。
            # 最后，将所有尺度的特征图沿通道维度进行拼接（torch.cat）。
            # 效果：
            # 拼接方式将所有尺度的特征都保留在最终的特征图中，能够更充分地利用不同尺度的信息，不过这种方式会增加输出的通道数。
        elif self.agg_mode == 'cat':
            x1 = self.gate(self.conv1(x1))  # B x dim//4 x H x W
            x2 = self.gate(self.conv2(x2))  # B x dim//4 x H//2 x W//2
            x3 = self.gate(self.conv3(x3))  # B x dim//4 x H//8 x W//8
            x4 = self.gate(self.conv4(x4))  # B x dim//4 x H//32 x W//32
            x2_up = self.upsample2(x2)  # B x dim//4 x H x W
            x3_up = self.upsample8(x3)  # B x dim//4 x H x W
            x4_up = self.upsample32(x4)  # B x dim//4 x H x W
            x1234 = torch.cat([x1, x2_up, x3_up, x4_up], dim=1)
        else:
            raise ValueError(f"Unkown aggregation mode: '{self.agg_mode}', should be 'sum' or 'cat'!")

        # ================================== detector and descriptor head
        if not self.single_head:
            x1234 = self.gate(self.convhead1(x1234))
        x = self.convhead2(x1234)  # B x dim+1 x H x W
        # 1. 1x1卷积的作用
        # 1x1卷积的作用是在不改变特征图的空间维度（即H和W）的情况下，通过卷积核的线性组合改变特征图的通道数。这个过程可以看作是对每个像素点位置的特征向量进行线性变换。
        # 2. 输出的dim + 1通道
        # 通过这个1x1卷积，特征图的通道数从dim增加到dim + 1，也就是说，输出特征图的最后一个通道将包含用于表示某种特征存在的置信度值（logits）。这些置信度值在未经过任何非线性激活之前，并不直接是概率。
        # 3. 从置信度值到概率
        # 为了将最后一个通道中的置信度值转化为概率，通常会使用sigmoid函数。这是因为sigmoid函数的输出范围在0到1之间，正好可以表示概率的含义：
        # scores_map = torch.sigmoid(x[:, -1, :, :]).unsqueeze(1)
        # x[:, -1, :, :]选择输出特征图的最后一个通道，该通道包含卷积操作生成的"原始"置信度值（logits）。
        # torch.sigmoid将这些logits转化为概率值。
        # unsqueeze(1)增加一个维度，使得score_map的形状为[N, 1, H, W]，这样就可以与其他通道的特征图保持一致的维度结构。
        # 为什么1x1卷积和sigmoid组合可以表示概率？
        # 线性组合：1x1卷积核对输入特征的线性组合，可以学到不同通道之间的关系，从而输出一个新的通道，这个通道的值可以解释为在该位置上某种特征的强度。
        # 激活函数：sigmoid函数将强度值映射到[0, 1]之间，这个范围可以很好地解释为概率，表示模型认为该位置上特定特征存在的可能性。
        # 总结
        # 通过1x1卷积，网络学到了一种表示特定特征存在性的强度（logits），而通过sigmoid函数，强度被归一化为[0, 1]之间的值，从而可以解释为概率。
        # 这就是为什么通过self.convhead2 = resnet.conv1x1(dim, dim + 1)以及后续的sigmoid操作可以得到概率的原因。
        descriptor_map = x[:, :-1, :, :]
        scores_map = torch.sigmoid(x[:, -1, :, :]).unsqueeze(1)

        return scores_map, descriptor_map


if __name__ == '__main__':
    from thop import profile

    net = ALNet(c1=16, c2=32, c3=64, c4=128, dim=128, agg_mode='cat', single_head=True)

    image = torch.randn(1, 3, 640, 480)
    flops, params = profile(net, inputs=(image,))
    print('{:<30}  {:<8} GFLops'.format('Computational complexity: ', flops / 1e9))
    print('{:<30}  {:<8} KB'.format('Number of parameters: ', params / 1e3))
