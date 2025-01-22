/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Initializer.h"

#include "Thirdparty/DBoW2/DUtils/Random.h"

#include "Optimizer.h"
#include "ORBmatcher.h"

#include<thread>

namespace ORB_SLAM2
{

Initializer::Initializer(const Frame &ReferenceFrame, float sigma, int iterations)
{
    mK = ReferenceFrame.mK.clone();

    mvKeys1 = ReferenceFrame.mvKeysUn;

    mSigma = sigma;
    mSigma2 = sigma*sigma;
    mMaxIterations = iterations;
}

bool Initializer::Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    mvKeys2 = CurrentFrame.mvKeysUn;

    mvMatches12.clear();
    mvMatches12.reserve(mvKeys2.size());
    mvbMatched1.resize(mvKeys1.size());
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    const int N = mvMatches12.size();

    // Indices for minimum set selection
    // 保存全部特征点索引用于随机采样的一部分
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);
    vector<size_t> vAvailableIndices;

    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // mMaxIterations 表示 RANSAC 算法的最大迭代次数，即我们会运行多少次迭代来尝试找到最佳模型。
    // vector<size_t>(8, 0) 表示每次迭代将抽取 8 个特征点索引，并初始化为 0。
    // 每一行 mvSets[i] 表示第 i 次迭代中抽取的 8 个特征点的索引。这些索引会用于拟合模型（如单应矩阵或基础矩阵）并进行验证。
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));
    //参数值 0 的含义： 这里将种子值设为 0，目的是得到一个固定的随机数序列。根据需求，这个值也可以改成其他数字，但只要保持不变，每次运行程序产生的随机序列就会一致。
    DUtils::Random::SeedRandOnce(0);

    for(int it=0; it<mMaxIterations; it++)
    {
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int randi = DUtils::Random::RandomInt(0,vAvailableIndices.size()-1);
            //std::cout<<"randi:"<<randi<<std::endl;
            int idx = vAvailableIndices[randi];

            mvSets[it][j] = idx;

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }
    }

    // Launch threads to compute in parallel a fundamental matrix and a homography
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    float SH, SF;
    cv::Mat H, F;

    // ref 是 C++ 标准库中的一个函数模板，位于 <functional> 头文件中，通常用于创建对对象的引用包装，以便在多线程或其他场景中将对象按引用传递给函数。下面解释该语句中 ref 的作用：
    // 在这个语句中，std::ref 的作用是将 vbMatchesInliersH、SH 和 H 作为引用传递给 thread 对象中的 FindHomography 方法，而不是值传递。具体来说，ref 的作用包括：
    // 避免拷贝：在标准情况下，std::thread 会尝试通过值传递将参数传给新线程调用的函数。然而，对于大型对象或数组而言，值传递会导致不必要的拷贝操作，降低效率。通过使用 ref 包装后，参数会按引用传递，不会进行拷贝，提升性能。
    // 维持原对象引用：如果希望在 FindHomography 函数中直接修改主线程中的 vbMatchesInliersH、SH 和 H 的值，按引用传递是必须的。ref 确保 FindHomography 中的修改直接作用于主线程的这些变量。
    // 语法要求：在 std::thread 中，如果想按引用传递变量，直接使用 & 不会起作用，必须通过 std::ref 或 std::cref（常量引用）来实现引用传递。
    thread threadH(&Initializer::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
    thread threadF(&Initializer::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

    // Wait until both threads have finished
    threadH.join();
    threadF.join();

    // Compute ratio of scores
    float RH = SH/(SH+SF);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    if(RH>0.40)
        return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    else //if(pF_HF>0.6)
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

    return false;
}


void Initializer::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
    const int N = mvMatches12.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2inv = T2.inv();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat H21i, H12i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(size_t j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        // 归一化与逆归一化：

        // T1 和 T2inv 是对点集进行归一化变换的矩阵，其中 T1 用于将图像1中的关键点归一化，T2inv 是对图像2的关键点进行逆归一化的矩阵。
        // Hn 是通过归一化后的点计算得到的单应性矩阵。由于在计算过程中，点被进行了归一化，因此矩阵 Hn 仅适用于归一化后的点。
        // H21i = T2inv * Hn * T1：这一步是将归一化条件下得到的单应性矩阵 Hn 转换回原始坐标系中。这个公式的物理意义是恢复从图像1到图像2的映射矩阵。具体过程是：
        // 通过 T1 将原始图像1中的点进行归一化。
        // 通过 Hn 进行透视变换。
        // 再通过 T2inv 将归一化的结果变换回原始图像2的坐标系。
        // 最终得到的 H21i 是应用于原始图像平面的单应性矩阵。
        // 双向变换：
        // H12i = H21i.inv()：这行代码的作用是求得单应性矩阵 H21i 的逆矩阵，表示从图像2到图像1的变换。
        // 物理意义上，H21i 表示从图像1到图像2的映射，而其逆矩阵 H12i 则表示从图像2到图像1的映射。通过求逆，可以方便地计算从图像2到图像1的对应关系。这种双向映射关系在多视角几何中非常重要，因为很多场景中我们需要验证图像匹配的相互一致性。
        // 总结
        // H21i = T2inv * Hn * T1：将归一化坐标下计算的单应性矩阵 Hn 转换为原始坐标下的单应性矩阵，适用于从图像1到图像2的映射。
        // H12i = H21i.inv()：求出 H21i 的逆矩阵，得到从图像2到图像1的映射矩阵。
        // 通过这两个步骤，可以建立图像1和图像2之间的双向映射关系，用于进一步的几何验证和匹配点的重投影误差计算。这种归一化和逆归一化的过程能够使得计算更加数值稳定，从而提高单应性矩阵估计的精度。
        H21i = T2inv*Hn*T1;
        H12i = H21i.inv();

        currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            H21 = H21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


void Initializer::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // Number of putative matches
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
    cv::Mat T2t = T2.t();

    // Best Results variables
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
    vector<cv::Point2f> vPn1i(8);
    vector<cv::Point2f> vPn2i(8);
    cv::Mat F21i;
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            vPn1i[j] = vPn1[mvMatches12[idx].first];
            vPn2i[j] = vPn2[mvMatches12[idx].second];
        }

        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        F21i = T2t*Fn*T1;

        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

        if(currentScore>score)
        {
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}


cv::Mat Initializer::ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2)
{
    //p2 = H21 * p2
    //一对点两个方程
    //u2(h7u1 + h8v1 + h9) = h1u1 + h2v1 + h3
    //v2(h7u1 + h8v1 + h9) = h4u1 + h5v1 + h6
    const int N = vP1.size();

    cv::Mat A(2*N,9,CV_32F);
    //2i行
    //-u1h4 - v1h5 - h3 + v2u1h7 + v2v1h8 + v2h9 = 0
    //2i+1行
    //h1u1 + h2v1 + h3 -u2u1h7 - u2v1h8 - u2h9 = 0
    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    cv::Mat u,w,vt;
    //A = u * w * vt
    //u vt是正交矩阵，w是一个对角矩阵对角线上的元素是A的奇异值
    // U 矩阵的含义：
    // U 是一个左奇异矩阵，它的列向量是 A 的列空间的正交基。这意味着 U 的列向量描述了 A 的列空间结构，但并不直接描述解空间。
    // V 矩阵的含义：
    // V 的列向量是 A^T A 的特征向量，即 A 的行空间的正交基。换句话说，V 的列描述了 A 的解空间和零空间。由于我们关心的是 A 的零空间（满足A⋅h=0 的向量h，因此我们需要 V^T 中的列向量来构建解。
    // 最小奇异值方向：
    // 在 W 中，奇异值的大小代表了 A 在不同方向的缩放强度。最小的奇异值方向表示 A 最接近零的方向，也就是方程最接近无解的方向。这一方向正是我们需要的解方向。而这个方向对应于

    // 即使矩阵是不可逆的，奇异值分解 (SVD) 依然可以进行。奇异值分解是一种通用的方法，适用于任何矩阵，无论它是方阵还是非方阵，是否满秩（即是否可逆）
    // 即使矩阵是不可逆的，它的奇异值分解依然成立。这是因为不可逆矩阵的奇异值分解只会在w矩阵中包含一个或多个零奇异值。具体来说：
    // 当 A不可逆时，它至少有一个奇异值为零。零奇异值表明 A 在某些方向上没有伸缩作用，因此这些方向上没有信息增益。
    // 奇异值分解的优点
    // 即使矩阵不可逆，SVD 仍然是一个很有用的工具，因为它允许我们：
    // 分析矩阵的秩：通过奇异值的数量和大小来判断矩阵的秩。不为零的奇异值的数量等于矩阵的秩。
    // 降维：通过去除较小或为零的奇异值，可以在信息损失较小的情况下对矩阵进行近似。
    // 求伪逆：对于不可逆的矩阵 A，可以通过 SVD 求其 Moore-Penrose 伪逆，这在求解最小二乘问题时非常有用。
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    return vt.row(8).reshape(0, 3);//矩阵V T的最后一行转化为 3×3的单应性矩阵H
}

cv::Mat Initializer::ComputeF21(const vector<cv::Point2f> &vP1,const vector<cv::Point2f> &vP2)
{
    const int N = vP1.size();

    cv::Mat A(N,9,CV_32F);

    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    cv::Mat u,w,vt;

    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    cv::Mat Fpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

    w.at<float>(2)=0;

    return  u*cv::Mat::diag(w)*vt;
}
// 通过计算重投影误差来检查两个图像之间的单应性矩阵 H21的准确性，并用来判断哪些匹配点是内点。
// 它的主要工作包括计算两个图像之间的匹配点在应用单应性变换后的重投影误差，并基于误差判断内点（inliers）和外点（outliers），最后返回一个得分来表示变换的好坏
float Initializer::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma)
{   
    const int N = mvMatches12.size();

    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 5.991;
    // sigma 代表的是测量噪声的标准差
    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in first image
        // x2in1 = H12*x2
        //2在1中
        // 计算从图像 2 中某点投影到图像 1 中时的归一化因子
        const float w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
        const float u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
        const float v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

        const float squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += th - chiSquare1;

        // Reprojection error in second image
        // x1in2 = H21*x1

        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

        const float chiSquare2 = squareDist2*invSigmaSquare;
        // chiSquare1 是重投影误差的卡方统计量，它的值是通过测量误差平方与噪声方差的比值计算得到的，重投影误差越小，chiSquare1 的值就越小，说明点的变换符合模型，匹配良好
        //th 的值为 5.991，这个值是基于 卡方分布的阈值。在卡方分布中，5.991 通常对应于自由度为 2、置信度为 95% 的情况。
        //这个阈值用于判断给定匹配点的重投影误差是否在可以接受的范围内。也就是说，如果 chiSquare1 小于或等于 5.991，那么我们有 95% 的信心认为该误差在合理范围内，说明这个点符合单应性变换
        // th - chiSquare1 表示将阈值与误差之间的差作为得分。误差越小，chiSquare1 就越小，th - chiSquare1 就越大，说明该匹配点越符合单应性变换模型，得分越高。
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

float Initializer::CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma)
{
    const int N = mvMatches12.size();

    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

    vbMatchesInliers.resize(N);

    float score = 0;

    const float th = 3.841;
    const float thScore = 5.991;

    const float invSigmaSquare = 1.0/(sigma*sigma);

    for(int i=0; i<N; i++)
    {
        bool bIn = true;

        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // l2=F21x1=(a2,b2,c2)

        const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;

        const float num2 = a2*u2+b2*v2+c2;

        const float squareDist1 = num2*num2/(a2*a2+b2*b2);

        const float chiSquare1 = squareDist1*invSigmaSquare;

        if(chiSquare1>th)
            bIn = false;
        else
            score += thScore - chiSquare1;

        // Reprojection error in second image
        // l1 =x2tF21=(a1,b1,c1)

        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        const float num1 = a1*u1+b1*v1+c1;

        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        const float chiSquare2 = squareDist2*invSigmaSquare;

        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;

        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }

    return score;
}

bool Initializer::ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                            cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // Compute Essential Matrix from Fundamental Matrix
    cv::Mat E21 = K.t()*F21*K;

    cv::Mat R1, R2, t;

    // Recover the 4 motion hypotheses
    DecomposeE(E21,R1,R2,t);  

    cv::Mat t1=t;
    cv::Mat t2=-t;

    // Reconstruct with the 4 hyphoteses and check
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
    float parallax1,parallax2, parallax3, parallax4;

    int nGood1 = CheckRT(R1,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D1, 4.0*mSigma2, vbTriangulated1, parallax1);
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    R21 = cv::Mat();
    t21 = cv::Mat();

    int nMinGood = max(static_cast<int>(0.9*N),minTriangulated);

    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // If there is not a clear winner or not enough triangulated points reject initialization
    if(maxGood<nMinGood || nsimilar>1)
    {
        return false;
    }

    // If best reconstruction has enough parallax initialize
    if(maxGood==nGood1)
    {
        if(parallax1>minParallax)
        {
            vP3D = vP3D1;
            vbTriangulated = vbTriangulated1;

            R1.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            return true;
        }
    }

    return false;
}

bool Initializer::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    //统计内点的数量，这些内点是经过初步检查后符合单应性矩阵的匹配点。
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988
    //n^T * P = d 个法向量n 和距离 d
    // 一个空间点P, 在相机1内的坐标为P_1, 在两个相机内的投影分别为
    // p1 = K * P1  p2 = K * (R21 * P1 + t21)
    // p2 = K * (R21 * P1 + t21 * ((n^T * P1) / d ))
    //    = K * (R21 + t21 * (n^T / d)) * K ^-1 * p1
    // H = K * (R21 + t21 * (n^T / d)) * K ^-1
    // A = (R21 + t21 * (n^T / d))
    //三维空间点满足 P2 = A * P1
    //为什么有的推导 A = (R21 * d + t21 * n^T) 尺度等价性
    // 尺度等价性约束（也叫比例约束或同一比例约束）是指在涉及齐次坐标的几何推导中，等式两边的表达式是比例等价的，而不是绝对相等的。
    // 也就是说，齐次坐标的表示允许在整体上乘以一个非零的比例因子，而几何意义保持不变。
    // 例如，在齐次坐标下，三维点 [X,Y,Z] 与 [λX,λY,λZ] 是等价的，因为它们表示的是同一个空间点，
    // 只是尺度因子不同而已。这个性质在投影几何中非常重要，因为在图像平面上的投影点并不保留绝对的深度信息。
    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    //A = U * w * VT
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();
    //determinant行列式
    //由于 U 和 V 都是正交矩阵，它们的行列式只会取 +1或−1，这意味着它们表示的是旋转或反射
    float s = cv::determinant(U)*cv::determinant(Vt);
    //d1 ≥ d2 ≥ d3 ≥ 0
    // 奇异值是由矩阵的自伴随矩阵（即 HT * H 或 H * HT）的特征值的平方根得到的。对于任意矩阵H，自伴随矩阵 HT * H 是一个半正定矩阵（即其特征值总是非负的）。
    // 这意味着，奇异值是这些特征值的平方根，因此也总是非负的。
    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }
    //H = U ∑ v^T
    //∑ = U^T * (R * d + t * n^T) * V = d * U^T * R * V + U^T * t * n^T * V
    //  = d * U^T * R * V + (U^T * t) * (V^T * n)^T =    (d1 0 0)
                                                    //   (0 d2 0)
                                                    //   (0 0 d3)
    // 令s = det(U)det(V) s^2 = 1
    //R' = s * U^T * R * V
    //d' = s * d
    //t' = (U^T * t)
    //n' = V^T * n
    //∑ = d' * R' + t' * n'^T
    //R'是旋转矩阵，乘了s det必为1（不是-1）， d'正负关系无法确定
    //取基底 e1 = (1,0,0) e2 = (0,1,0) e3 = (0,0,1)
    //n' = (x1,x2,x3)^T = x1 * e1 + x2 * e2 + x3 * e3;
    //∑ * (e1,e2,e3) = (d' * R' + t' * n'^T) * (e1,e2,e3)
    //(d1e1 d2e2 d3e3) = (d' * R' * e1 + t'x1, d' * R' * e2 + t'x2, d' * R' * e3 + t'x3)
    //写成三个方程组
    //d1e1 = d' * R' * e1 + t'x1
    //d2e2 = d' * R' * e2 + t'x2
    //d2e3 = d' * R' * e3 + t'x3
    //由于n是单位向量，V‘是旋转矩阵，所以n'也是单位向量
    //d' * R'（x2e1 - x1e2）= d1x2e1 - d2x1e2
    //d' * R' (x3e2 - x2e3) = d2x3e2 - d3x2e3
    //d' * R' (x1e3 - x3e1) = d3x1e3 - d1x3e1
    //R'是旋转矩阵不改变范数，e的范数为1，两边同时取二范数
    //（d'^2 - d2^2）x1^2 + (d'^2 - d1^2)x2^2 = 0
    //（d'^2 - d3^2）x2^2 + (d'^2 - d2^2)x3^2 = 0
    //（d'^2 - d1^2）x3^2 + (d'^2 - d3^2)x1^2 = 0
    //如果这个方程存在非0解，则系数矩阵必须是奇异矩阵，即其行列式为0
    //（d'^2 - d1^2）(d'^2 - d2^2)(d'^2 - d3^2) = 0
    // 根据H的奇异值大小关系d1 ≥ d2 ≥ d3 ≥ 0可以分为以下三种情况讨论
    //1 d' = ±d1  2 d' = ±d2  3 d' = ±d3
    //第一种情况d' = ±d1
    //（d'^2 - d2^2）x1^2 = 0
    //（d'^2 - d3^2）x2^2 + (d'^2 - d2^2)x3^2 = 0
    // (d'^2 - d3^2)x1^2 = 0
    // d1 > d2 只有x1 = x2 = x3 = 0 ∑xi = 1，故不成立
    // d1 = d2 x1 = 0 x2 = 0 x3 = ±1
    // d2 = d3 x1,x2,x3可以取任意值


    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}
// 三角化的目的是利用两个相机的投影矩阵 P1和 P2,从图像上的匹配点kp1和kp2反推出三维点X
// p = P * X
// u1 = (P1.row(0) * p1) / (P1.row(2)) * p1
// u1 * (P1.row(2)) * p1 - (P1.row(0) * p1) = 0
// (u1 * P1.row(2) - P1.row(0)) * X = 0
void Initializer::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

// 归一化点的坐标的主要目的是提高数值计算的稳定性和精度，尤其在处理几何变换（如单应性矩阵计算、相机姿态估计等）时，避免因坐标数值差异过大而引发的计算误差。通过将点的均值移动到原点，并统一它们的尺度，可以：
// 减少数值不稳定性：大范围的坐标值会导致计算过程中的数值误差，因此标准化坐标使得数值范围更加集中，有助于提高算法的稳定性。
// 增强算法的鲁棒性：对于涉及矩阵求逆、最小二乘法等数值优化算法，归一化处理可以减轻大值小值不均衡带来的影响，使得算法更不容易受到异常值的干扰。
// 提高计算效率：通过缩小坐标范围，可以避免在计算中因大范围数值导致的溢出或精度丢失，进而提升计算速度和精度。
// 总的来说，归一化操作让不同尺度的点集变得可比，从而提高了后续处理（如匹配、估计、优化等）的效果。
void Initializer::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
{
    // 计算点集的均值：
    float meanX = 0;
    float meanY = 0;
    const int N = vKeys.size();

    vNormalizedPoints.resize(N);

    for(int i=0; i<N; i++)
    {
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    meanX = meanX/N;
    meanY = meanY/N;
    // 计算点的标准差
    float meanDevX = 0;
    float meanDevY = 0;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    // 计算尺度因子
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }
    // 矩阵 T 是通过对点进行平移和缩放的组合变换，目的是将点集的均值移到原点，且使得每个坐标轴的尺度标准化
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}


int Initializer:: CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;
    //T_cam_world
    //Pcam = Rcam * Pworld + tWorld
    //pcam = 0
    //0 = Rcam * Pworld + tWorld
    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        // cos(Parallax) 越接近 1，视差角越小，说明两相机的基线很短，几何约束较弱。通过cos(Parallax)<0.99998 cos(Parallax)<0.99998 判断点的视差角是否足够大。如果角度过小，则跳过点的处理。
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void Initializer::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}

} //namespace ORB_SLAM
