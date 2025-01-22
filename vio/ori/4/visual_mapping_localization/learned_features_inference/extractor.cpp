#include "extractor.h"

struct greaterThanPtr
{
    bool operator()(const float *a, const float *b) const
    // Ensure a fully deterministic result of the sort
    {
        return (*a > *b) || *a >= *b && (a > b);
    }
};

/**
 * @brief TODO 根据OpenCV的goodFeaturesToTrack函数实现角点检测
 * @param score_map
 * @param cell_size
 * @param kps
 * @return
 */
// cv::InputArray 是 OpenCV 中的一个类型，用于处理各种不同类型的输入数据。它的主要目的是使 OpenCV 函数能够接受不同的数据类型而无需为每种数据类型编写多个重载版本。
std::vector<cv::KeyPoint> nms(cv::InputArray score_map, int maxCorners,
                              double qualityLevel, double minDistance, cv::InputArray _mask)
{
    std::vector<cv::KeyPoint> kps;
    CV_Assert(qualityLevel > 0 && minDistance >= 0 && maxCorners >= 0);
    CV_Assert(_mask.empty() || (_mask.type() == CV_8UC1 && _mask.sameSize(score_map)));

    cv::Mat eig = score_map.getMat(), tmp;
    double maxVal = 0;
    // eig：要计算最值的 cv::Mat 对象。
    // 0：表示不需要最小值（最小值的输出参数）。
    // &maxVal：指向一个 double 类型的指针，用于存储 eig 中的最大值。
    // 0：表示不需要最小值的位置（最小值位置的输出参数）。
    // 0：表示不需要最大值的位置（最大值位置的输出参数）。
    // _mask：一个可选的掩码，用于指定计算最值时哪些像素应该被考虑（非零值）。如果掩码为空，则会考虑所有像素。
    cv::minMaxLoc(eig, 0, &maxVal, 0, 0, _mask);
    std::cout << "maxVal:" << maxVal << std::endl;
    // 参数：
    // eig：输入图像和输出图像的 cv::Mat 对象。即对 eig 矩阵进行处理，并将结果存储在 eig 中。
    // maxVal * qualityLevel：阈值，所有大于这个值的像素将保留，其余像素将被设置为 0。
    // 0：表示在阈值化后设置的最大值。对于 cv::THRESH_TOZERO，这个值没有实际意义。
    // cv::THRESH_TOZERO：阈值化类型，所有小于阈值的像素将被设置为 0，所有大于阈值的像素保持不变。
    // 作用：
    // 将 eig 中所有小于 maxVal * qualityLevel 的像素值设为 0，其他像素值保持不变。这用于过滤掉低值区域，只保留那些较为显著的区域。
    cv::threshold(eig, eig, maxVal * qualityLevel, 0, cv::THRESH_TOZERO);
    // 对 eig 矩阵进行膨胀操作，以扩展图像中的高值区域。扩展 eig 中的高值区域，使这些区域的边缘变得更宽，这有助于增强特征区域的显著性，减少噪声干扰。
    // 参数：
    // eig：输入图像的 cv::Mat 对象。
    // tmp：输出图像的 cv::Mat 对象，用于存储膨胀后的结果。
    // cv::Mat()：结构元素，这里使用默认的 3x3 矩形结构元素（即一个空矩阵），OpenCV 会自动使用默认的结构元素进行膨胀。
    cv::dilate(eig, tmp, cv::Mat());
    //  cv::Size 对象，包含图像的宽度和高度。
    cv::Size imgsize = eig.size();
    // tmpCorners 将用来存储指向 eig 矩阵中非零元素的指针。这样可以快速访问这些元素，并对其进行进一步处理，比如进行非极大值抑制（NMS）等操作。
    std::vector<const float *> tmpCorners;
    // 从 eig 矩阵中提取非零的、局部最大值的特征点，同时考虑掩码 mask 的影响。
    cv::Mat mask = _mask.getMat();
    // 功能：
    // 遍历图像的每一行，排除边界行。
    // 作用：
    // y 是图像的行索引，从 1 开始到 imgsize.height - 1，以确保不处理边界行（因为要处理 y-1 和 y+1 行）。
    for (int y = 1; y < imgsize.height - 1; y++)
    {
        // 获取 eig 矩阵第 y 行的数据指针
        const auto *eig_data = (const float *)eig.ptr(y);
        // 获取 tmp 矩阵第 y 行的数据指针
        const auto *tmp_data = (const float *)tmp.ptr(y);
        // 获取 mask 矩阵第 y 行的数据指针，如果 mask 不为空。
        const uchar *mask_data = mask.data ? mask.ptr(y) : 0;
        // 遍历图像的每一列，排除边界列。
        for (int x = 1; x < imgsize.width - 1; x++)
        {
            float val = eig_data[x];
            // val != 0：确保值不为零。
            // val == tmp_data[x]：确保当前值是局部最大值（即在膨胀后的 tmp 矩阵中也是最大值）。
            // !mask_data || mask_data[x]：如果存在掩码，则掩码位置必须为真（即该位置应被考虑）。
            if (val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x]))
                // 将符合条件的像素地址添加到 tmpCorners 中
                tmpCorners.push_back(eig_data + x);
        }
    }
    // 用于存储特征点的质量评分。
    std::vector<float> cornersQuality;
    size_t i, j, total = tmpCorners.size(), ncorners = 0;

    if (total == 0)
    {
        return {};
    }

    std::sort(tmpCorners.begin(), tmpCorners.end(), greaterThanPtr());

    if (minDistance >= 1)
    {
        // Partition the image into larger grids
        int w = eig.cols;
        int h = eig.rows;
        // 将最小距离 minDistance 转换为整数，并赋值给 cell_size
        //  minDistance通常表示在非极大值抑制（NMS）过程中，相邻关键点之间的最小距离。
        const int cell_size = cvRound(minDistance);
        // 计算图像网格的宽度 (grid_width)，用于在后续的非极大值抑制（NMS）操作中，将图像划分为更大的网格单元，以便进行关键点的筛选。
        // 作用是进行上取整，确保即使图像宽度不能被整除，也能多分配一个网格单元。
        const int grid_width = (w + cell_size - 1) / cell_size;
        const int grid_height = (h + cell_size - 1) / cell_size;
        // grid_width * grid_height: 这部分计算了网格的总大小，即网格中包含的单元格（cell）的数量。
        //  每个单元格都是一个 std::vector<cv::Point2f>，用于存储在该单元格中的角点。
        std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

        minDistance *= minDistance;

        for (i = 0; i < total; i++)
        {
            // tmpCorners[i]:
            // tmpCorners 是一个 std::vector<const float*> 类型的容器，存储了每个有效角点在 eig 矩阵中的指针位置。tmpCorners[i] 表示第 i 个角点的指针。
            // eig.ptr():
            // eig.ptr() 返回的是矩阵 eig 第 0 行的起始指针，也就是整个矩阵数据的起始地址。
            // (const uchar*)tmpCorners[i] - eig.ptr():
            // 这部分计算了角点指针 tmpCorners[i] 与矩阵起始地址 eig.ptr() 之间的偏移量，单位是字节。
            // (int) 强制类型转换:
            // 偏移量最终被强制转换为 int 类型，表示该角点在矩阵数据中的线性索引
            int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
            // eig.step 表示每行的数据大小（以字节为单位）。
            // y 是行索引，x 是列索引。
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y * eig.step) / sizeof(float));

            bool good = true;
            // 在进行非极大值抑制（NMS）时，为了提高效率，我们通常会将图像划分成大小为 cell_size × cell_size 的网格（grid）。
            // 然后，每个角点会被分配到它所在的网格单元中。在进行距离检查时，只需要在该角点所在的网格及其周围的邻居网格中进行比较，减少了计算量。
            int x_cell = x / cell_size;
            int y_cell = y / cell_size;

            // 确定当前角点所在网格的邻居网格范围，用于后续在这些邻居网格中检查是否存在距离过近的角点。

            int x1 = x_cell - 1;
            int y1 = y_cell - 1;
            int x2 = x_cell + 1;
            int y2 = y_cell + 1;

            // boundary check
            x1 = std::max(0, x1);
            y1 = std::max(0, y1);
            x2 = std::min(grid_width - 1, x2);
            y2 = std::min(grid_height - 1, y2);

            for (int yy = y1; yy <= y2; yy++)
            {
                for (int xx = x1; xx <= x2; xx++)
                {
                    std::vector<cv::Point2f> &m = grid[yy * grid_width + xx];

                    if (m.size())
                    {
                        for (j = 0; j < m.size(); j++)
                        {
                            float dx = x - m[j].x;
                            float dy = y - m[j].y;

                            if (dx * dx + dy * dy < minDistance)
                            {
                                good = false;
                                goto break_out;
                            }
                        }
                    }
                }
            }

        break_out:

            if (good)
            {
                grid[y_cell * grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));

                cornersQuality.push_back(*tmpCorners[i]);
                kps.emplace_back((float)x, (float)y, 1.f);
                ++ncorners;

                if (maxCorners > 0 && (int)ncorners == maxCorners)
                    break;
            }
            // std::cout<<"ncorners:"<<ncorners<<std::endl;
        }
    }
    else
    {
        for (i = 0; i < total; i++)
        {
            cornersQuality.push_back(*tmpCorners[i]);

            int ofs = (int)((const uchar *)tmpCorners[i] - eig.ptr());
            int y = (int)(ofs / eig.step);
            int x = (int)((ofs - y * eig.step) / sizeof(float));

            kps.emplace_back((float)x, (float)y, 1.f);
            ++ncorners;

            if (maxCorners > 0 && (int)ncorners == maxCorners)
                break;
        }
    }

    return kps;
}

// 实现了一个双线性插值函数，用于在desc_map中根据给定的关键点（kps）获取特征描述符。函数的输入包括图像的宽度和高度（image_w和image_h），
// 特征描述符映射（desc_map）以及关键点的向量（kps）。输出是一个新的cv::Mat，包含每个关键点的描述符。
// 输入参数：
// image_w 和 image_h: 原始图像的宽度和高度。
// desc_map: 特征描述符映射，通常是卷积神经网络输出的特征图。
// kps: 一组关键点（cv::KeyPoint），每个关键点都有一个位置（kp.pt）。
// 输出：
// 一个大小为(kps.size(), c)的cv::Mat，其中c是desc_map的通道数。该矩阵包含每个关键点的描述符。
cv::Mat bilinear_interpolation(int image_w, int image_h, const cv::Mat &desc_map,
                               const std::vector<cv::KeyPoint> &kps)
{
    int w = desc_map.cols;
    int h = desc_map.rows;
    int c = desc_map.channels();
    cv::Mat desc((int)kps.size(), c, CV_32F);
    for (int i = 0; i < kps.size(); i++)
    {
        cv::KeyPoint kp = kps[i];
        // kp.pt.x / (float)image_w: 计算出关键点的x坐标在原始图像中的归一化位置（范围是0到1）。
        // 乘以(float)w：将归一化的位置缩放到特征图的范围内，得到关键点在特征图空间中的x坐标。
        // y的计算方法与x类似，是对y坐标的映射。
        float x = kp.pt.x / (float)image_w * (float)w;
        float y = kp.pt.y / (float)image_h * (float)h;
        // x 和 y 是之前计算出的关键点在特征图中的浮点坐标。
        // x0 和 y0 是将x和y转换为整数后的结果（取整部分），即关键点坐标在特征图中的左上角像素的整数索引。
        int x0 = (int)x;
        int y0 = (int)y;
        // x1 和 y1 分别是x0和y0右边和下面的像素坐标。
        // x1 = x0 + 1：这是右侧相邻像素的x坐标。
        // y1 = y0 + 1：这是下方相邻像素的y坐标。
        // 使用 std::min() 函数确保 x1 和 y1 不会超出特征图的边界（w - 1 和 h - 1），防止越界访问
        int x1 = std::min(x0 + 1, w - 1);
        int y1 = std::min(y0 + 1, h - 1);
        float dx = x - (float)x0;
        float dy = y - (float)y0;
        // 用于从desc_map（描述符特征图）中获取指定关键点的特征描述符。
        // c 是特征图desc_map的通道数（通常是特征描述符的维度）。
        // 对于每一个关键点kp，需要计算所有通道的插值值，因此使用一个循环来遍历每个通道j。
        for (int j = 0; j < c; j++)
        {
            float v00 = desc_map.at<float>(y0, x0 * c + j);
            float v01 = desc_map.at<float>(y1, x0 * c + j);
            float v10 = desc_map.at<float>(y0, x1 * c + j);
            float v11 = desc_map.at<float>(y1, x1 * c + j);
            desc.at<float>(i, j) = (1 - dx) * (1 - dy) * v00 + dx * (1 - dy) * v10 +
                                   (1 - dx) * dy * v01 + dx * dy * v11;
        }
    }

    return desc;
}
