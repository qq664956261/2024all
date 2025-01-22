//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam
{

    Frontend::Frontend()
    {
        gftt_ =
            //     cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
            // num_features_init_ = Config::Get<int>("num_features_init");
            // num_features_ = Config::Get<int>("num_features");
            cv::GFTTDetector::create(150, 0.01, 20);
        num_features_init_ = 50;
        num_features_ = 150;
    }

    bool Frontend::AddFrame(myslam::Frame::Ptr frame)
    {
        current_frame_ = frame;

        switch (status_)
        {
        case FrontendStatus::INITING:
            std::cout << "INITING" << std::endl;
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            std::cout << "TRACKING_GOOD" << std::endl;
            Track();
            break;
        case FrontendStatus::LOST:
            std::cout << "LOST" << std::endl;
            Reset();
            break;
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Track()
    {
        if (last_frame_)
        {
            current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
        }

        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();
        std::cout << "tracking_inliers_:" << tracking_inliers_ << std::endl;
        if (tracking_inliers_ > num_features_tracking_)
        {
            // tracking good
            status_ = FrontendStatus::TRACKING_GOOD;
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            // tracking bad
            status_ = FrontendStatus::TRACKING_BAD;
        }
        else
        {
            // lost
            status_ = FrontendStatus::LOST;
        }

        InsertKeyframe();
        relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();

        if (viewer_)
            viewer_->AddCurrentFrame(current_frame_);
        return true;
    }

    bool Frontend::InsertKeyframe()
    {
        if (tracking_inliers_ >= num_features_needed_for_keyframe_)
        {
            // still have enough features, don't insert keyframe
            return false;
        }
        // current frame is a new keyframe
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
                  << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();
        DetectFeatures(); // detect new features

        // track in right image
        FindFeaturesInRight();
        // triangulate map points
        TriangulateNewPoints();
        // update backend because we have a new keyframe
        backend_->UpdateMap();

        if (viewer_)
            viewer_->UpdateMap();

        return true;
    }

    void Frontend::SetObservationsForKeyFrame()
    {
        for (auto &feat : current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        SE3 current_pose_Twc = current_frame_->Pose().inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->position_.pt.x,
                             current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->position_.pt.x,
                             current_frame_->features_right_[i]->position_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                if (triangulation(poses, points, pworld) && pworld[2] > 0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(
                        current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                        current_frame_->features_right_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    int Frontend::EstimateCurrentPose()
    {
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(
                g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // vertex
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.addVertex(vertex_pose);

        // K
        Mat33 K = camera_left_->K();

        // edges
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp)
            {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->pos_, K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // estimate the Pose the determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        for (int iteration = 0; iteration < 4; ++iteration)
        {
            vertex_pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
                  << features.size() - cnt_outlier;
        // Set pose and outlier
        current_frame_->SetPose(vertex_pose->estimate());

        LOG(INFO) << "Current Pose = \n"
                  << current_frame_->Pose().matrix();

        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                feat->map_point_.reset();
                feat->is_outlier_ = false; // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier;
    }

    int Frontend::TrackLastFrame()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                // use project point
                auto mp = kp->map_point_.lock();
                auto px =
                    camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;

        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.push_back(feature);
                num_good_pts++;
            }
        }

        LOG(INFO) << "Find " << num_good_pts << " in the last image.";
        return num_good_pts;
    }

    bool Frontend::StereoInit()
    {
        int num_features_left = DetectFeatures();
        int num_coor_features = FindFeaturesInRight();
        std::cout << "num_features_left:" << num_features_left << std::endl;
        std::cout << "num_coor_features:" << num_coor_features << std::endl;
        if (num_coor_features < num_features_init_)
        {
            std::cout << "num_coor_features:" << num_coor_features << std::endl;
            std::cout << "num_features_init_:" << num_features_init_ << std::endl;
            return false;
        }

        bool build_map_success = BuildInitMap();
        std::cout << "build_map_success:" << build_map_success << std::endl;
        if (build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_)
            {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }

            return true;
        }
        else
        {
            std::cout << "build map failed!!!!!!!!!!! " << std::endl;
        }
        return false;
    }

    int Frontend::DetectFeatures()
    {
        // 创建一个 cv::Mat 类型的掩码（mask），它的具体功能是创建一个与当前帧的左图像 (left_img_) 大小相同的掩码图像，初始时每个像素的值都是 255
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_)
        {
            // cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
            //               feat->position_.pt + cv::Point2f(10, 10), 0, cv::FILLED);
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(5, 5),
                          feat->position_.pt + cv::Point2f(5, 5), 0, cv::FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        // mask: 这是一个 cv::Mat 类型的掩码图像，用于指定在图像的哪些区域进行特征点检测。掩码图像通常是一个与输入图像大小相同的单通道图像，其中掩码值为 255 的区域表示检测区域，值为 0 的区域表示不进行检测。
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        // gftt_->detect(current_frame_->left_img_, keypoints);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected++;
        }


        LOG(INFO) << "Detect " << cnt_detected << " new features";
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);
            // 目的是从一个 std::weak_ptr 获取一个 std::shared_ptr。
            // 如果该 std::weak_ptr 指向的对象仍然存在，则 lock() 返回一个指向该对象的 std::shared_ptr；否则返回一个空的 std::shared_ptr。
            auto mp = kp->map_point_.lock();
            if (mp)
            {
                // use projected points as initial guess
                auto px =
                    camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                // use same pixel in left iamge
                kps_right.push_back(kp->position_.pt);
            }
        }

        std::vector<uchar> status;
        Mat error;
        // prevImg: 前一帧的图像。
        // nextImg: 当前帧的图像。
        // prevPts: 前一帧中的特征点（关键点）。
        // nextPts: 输出当前帧中的特征点（关键点）。
        // status: 一个向量，用于指示每个点是否成功跟踪。如果是 1 表示成功跟踪，0 表示未跟踪。
        // err: 一个向量，包含每个点的跟踪误差。
        // winSize: 搜索窗口的大小。
        // maxLevel: 使用的图像金字塔层数。
        // criteria: 迭代的终止条件。
        // flags: 可选标志：
        // cv::OPTFLOW_USE_INITIAL_FLOW 使用初始估计值。
        // cv::OPTFLOW_LK_GET_MIN_EIGENVALS 获取每个像素区域的最小特征值。
        // minEigThreshold: 计算光流时的最小特征值阈值。
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                // _pt：关键点的坐标。这里是 kps_right[i]，即跟踪后的关键点坐标。
                // _size：关键点的直径。
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_, kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        LOG(INFO) << "Find " << num_good_pts << " in the right image.";
        return num_good_pts;
    }

    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            // create map point from triangulation
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();
            if (triangulation(poses, points, pworld) && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                cnt_init_landmarks++;
                map_->InsertMapPoint(new_map_point);
            }
            else
            {
                std::cout << "triangulation failed" << std::endl;
                std::cout << "pworld:" << pworld << std::endl;
            }
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap();

        LOG(INFO) << "Initial map created with " << cnt_init_landmarks
                  << " map points";

        return true;
    }

    bool Frontend::Reset()
    {
        LOG(INFO) << "Reset is not implemented. ";
        return true;
    }

} // namespace myslam