import torch
import numpy as np


class SuperPointNet(torch.nn.Module):
    """ Pytorch definition of SuperPoint Network. """

    def __init__(self):
        super(SuperPointNet, self).__init__()
        # TODO 添加SP的网络结构
        self.relu = torch.nn.ReLU(inplace=True)
        self.pool = torch.nn.MaxPool2d(kernel_size=2, stride=2)
        c1, c2, c3, c4, c5, d1 = 64, 64, 128, 128, 256, 256
        self.conv1a = torch.nn.Conv2d(1, c1, kernel_size=3, stride=1, padding=1)
        self.conv1b = torch.nn.Conv2d(c1, c1, kernel_size=3, stride=1, padding=1)
        self.conv2a = torch.nn.Conv2d(c1, c2, kernel_size=3, stride=1, padding=1)
        self.conv2b = torch.nn.Conv2d(c2, c2, kernel_size=3, stride=1, padding=1)
        self.conv3a = torch.nn.Conv2d(c2, c3, kernel_size=3, stride=1, padding=1)
        self.conv3b = torch.nn.Conv2d(c3, c3, kernel_size=3, stride=1, padding=1)
        self.conv4a = torch.nn.Conv2d(c3, c4, kernel_size=3, stride=1, padding=1)
        self.conv4b = torch.nn.Conv2d(c4, c4, kernel_size=3, stride=1, padding=1)
        # Detector Head.
        self.convPa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convPb = torch.nn.Conv2d(c5, 65, kernel_size=1, stride=1, padding=0)
        # Descriptor Head.
        self.convDa = torch.nn.Conv2d(c4, c5, kernel_size=3, stride=1, padding=1)
        self.convDb = torch.nn.Conv2d(c5, d1, kernel_size=1, stride=1, padding=0)
        

    def forward(self, x):
        """ Forward pass that jointly computes unprocessed point and descriptor
    tensors.
    Input
      x: Image pytorch tensor shaped N x 1 x H x W.
    Output
      semi: Output point pytorch tensor shaped N x 65 x H/8 x W/8.
      desc: Output descriptor pytorch tensor shaped N x 256 x H/8 x W/8.
    """ 
        print("Type of x:", type(x))
        print("x.shape:", x.shape)
        B, C, H, W = x.shape
        Hc = int(H / 8)
        Wc = int(W / 8)
        x = torch.sum(x, dim=1, keepdim=True) # 把输入的图像转换为灰度图
        print("2x.shape:", x.shape)
        print("2Type of x:", type(x))
        # Shared Encoder.
        # TODO 添加SP的网络结构
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
        print("3x.shape:", x.shape)
        print("3Type of x:", type(x))
        # Detector Head.
        # 使用卷积层生成点特征图
        cPa = self.relu(self.convPa(x))  # (N x c4 x H/8 x W/8) -> (N x c5 x H/8 x W/8)
        semi = self.convPb(cPa)          # (N x c5 x H/8 x W/8) -> (N x 65 x H/8 x W/8)
        print("semi.shape:", semi.shape)
        print("semi of x:", type(semi))
        
        # Descriptor Head.
        # 使用卷积层生成描述符特征图
        cDa = self.relu(self.convDa(x))  # (N x c4 x H/8 x W/8) -> (N x c5 x H/8 x W/8)
        desc = self.convDb(cDa)          # (N x c5 x H/8 x W/8) -> (N x d1 x H/8 x W/8)
        print("desc.shape:", desc.shape)
        print("type of desc:", type(desc))
        
        # 归一化描述符
        dn = torch.norm(desc, p=2, dim=1)  # 计算描述符的 L2 范数: (N x d1 x H/8 x W/8) -> (N x 1 x H/8 x W/8)
        desc = desc.div(torch.unsqueeze(dn, 1))  # 将描述符除以其范数进行归一化: (N x d1 x H/8 x W/8)
        print("2desc.shape:", desc.shape)
        print("2type of desc:", type(desc))


        semi = semi.data.cpu().numpy().squeeze()

        # 将网络的原始输出转换为概率分布,这里使用了 softmax 函数将输出转换为概率
        dense = np.exp(semi) # Softmax.
        # 这一步是对 dense 进行归一化处理，使得每个位置的概率和为 1。
        dense = dense / (np.sum(dense, axis=0)+.00001) # Should sum to 1.
        # Remove dustbin.
        nodust = dense[:-1, :, :]
        # Reshape to get full resolution heatmap.
        # 由于特征图的每个单元大小为 self.cell x self.cell（这里是 8x8 像素），因此：
        # 特征图的高度 Hc 是原始图像高度 H 除以每个单元的大小 self.cell。
        # 特征图的宽度 Wc 是原始图像宽度 W 除以每个单元的大小 self.cell。
        # 这样计算出来的 Hc 和 Wc 就是网络输出特征图的分辨率，它们反映了每个特征点在原始图像中的位置。
        self.cell = 8
        # nodust 是一个三维数组，其原始形状是 (num_classes, Hc, Wc)，其中 num_classes 是特征图中的类别数（不包括最后的“垃圾桶”类别）。
        # transpose 方法将其维度调整为 (Hc, Wc, num_classes)
        nodust = nodust.transpose(1, 2, 0)
        # 将 nodust 重塑为一个四维数组 heatmap，其形状是 (Hc, Wc, self.cell, self.cell)。
        # 这一步是将特征图的每个单元（cell）从一个平面（2D）扩展为一个小的 2D 区域，这个区域对应于实际图像中的 8x8 像素块。
        heatmap = np.reshape(nodust, [Hc, Wc, self.cell, self.cell])
        # transpose 操作将 heatmap 的维度从 (Hc, Wc, self.cell, self.cell) 变为 (Hc, self.cell, Wc, self.cell)。这一步是为了将每个单元的区域进行正确的排列，方便下一步的重塑操作。
        heatmap = np.transpose(heatmap, [0, 2, 1, 3])
        # 将 heatmap 重塑为一个二维数组，其形状是 (Hc*self.cell, Wc*self.cell)，即 (H, W)。这将特征图的分辨率恢复到接近原始图像的分辨率。
        heatmap = np.reshape(heatmap, [1,1,Hc*self.cell, Wc*self.cell])
        heatmap = torch.from_numpy(heatmap)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        heatmap = heatmap.to(device)



        # semi = semi.squeeze(0)
        # dense = torch.exp(semi)/(torch.sum(torch.exp(semi),axis=0) * 0.0001)
        # nodust = dense[:-1,:,:].permute(1,2,0)
        # heatmap = nodust.reshape(Hc, Wc, 8, 8).permute(0,2,1,3).reshape(1,1,Hc*8,Wc*8)
        print("heatmap.shape:", heatmap.shape)
        print("type of heatmap:", type(heatmap))
        

        return heatmap, desc


class SuperPoint(object):
    def __init__(self, weights_path):
        self.net = SuperPointNet()
        self.net.load_state_dict(torch.load(weights_path, map_location='cpu'))
        self.net.eval()

    def __call__(self, img):
        """
        :param img: [B, 1, H, W] H, W % 64 == 0 in range [0,1].
        :return:  score map        [B, 1, H, W] in range [0,1].
                  local desc map 0 [B, 3, H, W]
                  local desc map 1 [B, 16, H/8, W/8] normalized
                  desc map         [B, 32, H/64, W/64]
        """
        B, C, H, W = img.shape
        assert H % 64 == 0 and W % 64 == 0
        if C == 3:
            img = torch.mean(img, dim=1, keepdim=True)
        with torch.no_grad():
            semi, desc = self.net(img)
        semi = semi.data.cpu().numpy().squeeze()
        dense = np.exp(semi) / (np.sum(np.exp(semi), axis=0) + 0.0001)
        nodust = dense[:-1, :, :].transpose(1, 2, 0)
        Hc = int(H / 8)
        Wc = int(W / 8)
        heatmap = np.reshape(nodust, [Hc, Wc, 8, 8])
        heatmap = np.transpose(heatmap, [0, 2, 1, 3])
        heatmap = np.reshape(heatmap, [Hc * 8, Wc * 8])
        desc_map_0 = torch.cat((img, torch.cat((img, img), dim=1)), dim=1)
        desc_map_0 = desc_map_0 / (torch.norm(desc_map_0, p=2, dim=1, keepdim=True) + 0.0001)
        desc_map_1 = desc[:, 0:4, :, :]
        desc_map_1 = desc_map_1 / (torch.norm(desc_map_1, p=2, dim=1, keepdim=True) + 0.0001)
        Hc = int(H / 64)
        Wc = int(W / 64)
        desc_map_2 = torch.zeros(1, 256, Hc, Wc)
        for i in range(Hc):
            for j in range(Wc):
                desc_map_2[:, :, i, j] = torch.mean(desc[:, :256, i * 8:(i + 1) * 8, j * 8:(j + 1) * 8], dim=(2, 3))
        desc_map_2 = desc_map_2 / (torch.norm(desc_map_2, p=2, dim=1, keepdim=True) + 0.0001)
        return torch.from_numpy(heatmap).unsqueeze(0).unsqueeze(0), desc_map_0, desc_map_1, desc_map_2


if __name__ == '__main__':
    from thop import profile
    import numpy as np
    net = SuperPointNet()
    net.load_state_dict(torch.load('/home/zc/code/vio/tast_1/weights/superpoint_v1.pth', map_location='cpu'))
    net.eval()
    image = torch.tensor(np.random.random((1, 1, 512, 512)), dtype=torch.float32)
    flops, params = profile(net, inputs=(image,))
    print('{:<30}  {:<8} GFLops'.format('Computational complexity: ', flops / 1e9))
    print('{:<30}  {:<8} KB'.format('Number of parameters: ', params / 1e3))

