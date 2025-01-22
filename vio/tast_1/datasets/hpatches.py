import numpy as np
import torch.utils.data as data
import glob
from pathlib import Path
import cv2


class HPatchesDataset(data.Dataset):
    def __init__(self,
                 root: str = '../data/hpatches',
                 alteration: str = 'all',
                 image_size: int = 512,
                 gray: bool = False):
        """
        Args:
            root: dataset root path
            alteration: # 'all', 'i' for illumination or 'v' for viewpoint
        """
        assert (Path(root).exists()), f"Dataset root path {root} dose not exist!"
        self.root = root
        self.image_size = image_size
        self.gray = gray
        # get all image file name
        self.image0_list = []
        self.image1_list = []
        self.homographies = []
        # Path(self.root).iterdir(): 使用 Path 对象来表示 self.root 路径，并调用 iterdir() 方法生成一个迭代器，该迭代器会生成该目录下的所有文件和子目录。
        # [x for x in ... if x.is_dir()]: 使用列表生成式 (list comprehension) 遍历这个迭代器，并筛选出其中所有的目录（使用 is_dir() 方法检查）。
        folders = [x for x in Path(self.root).iterdir() if x.is_dir()]
        for folder in folders:
            if alteration == 'i' and folder.stem[0] != 'i':#查当前子目录名称的第一个字符是否为 'i'
                continue
            if alteration == 'v' and folder.stem[0] != 'v':#检查当前子目录名称的第一个字符是否为 'v'
                continue
            # count images
            file_ext = '.ppm'
            #这行代码生成一个匹配模式，用于查找当前子目录中所有扩展名为 .ppm 的文件。folder 是当前处理的子目录，'*' + file_ext 会匹配所有 .ppm 文件。folder / ('*' + file_ext) 会生成类似于 folder/*.ppm 的路径模式
            pattern = folder / ('*' + file_ext)
            #代码使用 glob 模块查找所有与生成的路径模式匹配的文件。glob.glob(pathname=str(pattern)) 会返回一个包含所有匹配文件路径的列表
            img_names = glob.glob(pathname=str(pattern))
            #统计匹配的文件数量，即图像文件的数量
            num_images = len(img_names)
            # get image pair file names and homographies
            # 生成一个从2到num_images的索引范围。这样可以构建图像对，第一个图像固定为 1.ppm，第二个图像依次为 2.ppm 到 num_images.ppm
            for i in range(2, 1 + num_images):
                self.image0_list.append(str(Path(folder, '1' + file_ext)))
                self.image1_list.append(str(Path(folder, str(i) + file_ext)))
                self.homographies.append(str(Path(folder, 'H_1_' + str(i))))

        self.len = len(self.image0_list)
        assert (self.len > 0), f'Can not find PatchDataset in path {self.root}'

    def __getitem__(self, item):
        # read image
        img0 = cv2.imread(self.image0_list[item], cv2.IMREAD_COLOR)
        img1 = cv2.imread(self.image1_list[item])
        assert img0 is not None, 'can not load: ' + self.image0_list[item]
        assert img1 is not None, 'can not load: ' + self.image1_list[item]
        # 将图像从BGR格式转换为灰度图像，并进行归一化处理。如果不需要转换为灰度图像，则将图像从BGR格式转换为RGB格式，并进行归一化处理
        if self.gray:
            img0 = cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY).astype('float32') / 255.  # HxWx1
            img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY).astype('float32') / 255.  # HxWx1
            img0 = np.expand_dims(img0, axis=2)#在灰度图像的最后一个维度添加一个通道维度，使其形状从 (H, W) 变为 (H, W, 1)
            img1 = np.expand_dims(img1, axis=2)
        else:
            # bgr -> rgb
            img0 = cv2.cvtColor(img0, cv2.COLOR_BGR2RGB).astype('float32') / 255.  # HxWxC
            img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB).astype('float32') / 255.  # HxWxC

        h0, w0, _ = img0.shape
        h1, w1, _ = img1.shape
        # resize image
        img0 = cv2.resize(img0, (self.image_size, self.image_size))
        img1 = cv2.resize(img1, (self.image_size, self.image_size))
        # read homography
        homography = np.loadtxt(self.homographies[item]).astype('float32')
        if self.gray:
            img0 = np.expand_dims(img0, axis=2)
            img1 = np.expand_dims(img1, axis=2)
        # pack return dict
        #  PyTorch中，通常期望输入张量的形状为 [C, H, W]，即通道数在第一个维度，图像的高度和宽度分别在第二和第三个维度。
        # 而 OpenCV 读取的图像的默认格式是 [H, W, C]，即高度和宽度在前，通道数在最后一个维度。因此，需要进行维度的变换，即 transpose 操作，将图像从 [H, W, C] 格式转换为 [C, H, W] 格式。
        return {'image0': img0.transpose(2, 0, 1),  # [C,H,W]
                'image1': img1.transpose(2, 0, 1),  # [C,H,W]
                'warp01_params': {'mode': 'homo', 'width': w1, 'height': h1,
                                  'homography_matrix': homography,
                                  'resize': self.image_size},
                'warp10_params': {'mode': 'homo', 'width': w0, 'height': h0,
                                  'homography_matrix': np.linalg.inv(homography),
                                  'resize': self.image_size},
                'dataset': 'HPatches'
                }

    def __len__(self):
        return self.len

    def name(self):
        return self.__class__


if __name__ == '__main__':
    from tqdm import tqdm
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('TkAgg')

    hpatches_dataset = HPatchesDataset(root='/home/zc/code/vio/tast_1/data/hpatches', alteration='i', image_size = 512)
    for data in tqdm(hpatches_dataset):
        image0 = data['image0']
        image1 = data['image1']
        plt.imshow(image0.transpose(1, 2, 0))
        plt.show()
