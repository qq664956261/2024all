#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam
{

    Dataset::Dataset(const std::string &dataset_path)
        : dataset_path_(dataset_path) { std::cout << "dataset_path_:" << dataset_path_ << std::endl; }

    bool Dataset::Init()
    {
        if (use_kitti_)
        {
            // read camera intrinsics and extrinsics
            ifstream fin(dataset_path_ + "/calib.txt");
            if (!fin)
            {
                LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
                return false;
            }

            for (int i = 0; i < 4; ++i)
            {
                char camera_name[3];
                for (int k = 0; k < 3; ++k)
                {
                    fin >> camera_name[k];
                }
                double projection_data[12];
                for (int k = 0; k < 12; ++k)
                {
                    fin >> projection_data[k];
                }
                Mat33 K;
                K << projection_data[0], projection_data[1], projection_data[2],
                    projection_data[4], projection_data[5], projection_data[6],
                    projection_data[8], projection_data[9], projection_data[10];
                Vec3 t;
                t << projection_data[3], projection_data[7], projection_data[11];
                t = K.inverse() * t;
                K = K * 0.5;
                Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                                  t.norm(), SE3(SO3(), t)));
                cameras_.push_back(new_camera);
                LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
            }
            fin.close();
            current_image_index_ = 0;
        }
        else if (use_euroc_)
        {
            Mat33 K;
            K << 461.6, 0, 363,
                0, 460.3, 248.1,
                0, 0, 1;
            Vec3 t1, t2;
            t1 << 0, 0, 0;
            t2 << 0.11, 0, 0;
            Camera::Ptr new_camera1(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                               t1.norm(), SE3(SO3(), t1)));
            Camera::Ptr new_camera2(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                               t2.norm(), SE3(SO3(), t2)));
            cameras_.push_back(new_camera1);
            cameras_.push_back(new_camera2);
            current_image_index_ = 0;
            std::string filenames = dataset_path_ + "/cam0/data.csv";

            std::ifstream infile(filenames.c_str());
            for (std::string line; std::getline(infile, line);)
            {
                std::stringstream ss(line);
                std::string str;
                std::vector<std::string> lineArray;
                std::vector<double> data;
                while (getline(ss, str, ','))
                {
                    lineArray.push_back(str);
                }
                // std::cout << "lineArray:" << lineArray[1] << std::endl;
                image_names_.push_back(lineArray[0] + ".png");
                // std::cout<<"image_names_.size():"<<image_names_.size()<<std::endl;
            }
        }
        return true;
    }

    Frame::Ptr Dataset::NextFrame()
    {
        boost::format fmt("%s/image_%d/%06d.png");
        cv::Mat image_left, image_right;
        // read images
        if (use_kitti_)
        {
            image_left =
                cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                           cv::IMREAD_GRAYSCALE);
            image_right =
                cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                           cv::IMREAD_GRAYSCALE);
        }
        else if (use_euroc_)
        {
            std::string left_string = dataset_path_ + "/cam0/data/" + image_names_[current_image_index_];
            std::string right_string = dataset_path_ + "/cam1/data/" + image_names_[current_image_index_];
            image_left = cv::imread(left_string,
                                    cv::IMREAD_GRAYSCALE);
            image_right = cv::imread(right_string,
                                     cv::IMREAD_GRAYSCALE);
            // std::cout << dataset_path_ + "/cam1/data/" + image_names_[current_image_index_] << std::endl;
            // std::cout<<"left_string:"<<left_string<<std::endl;
        }

        if (image_left.data == nullptr || image_right.data == nullptr)
        {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            std::cout << image_names_[current_image_index_] << std::endl;
            return nullptr;
        }

        cv::Mat image_left_resized, image_right_resized;
        // 图像的大小调整为原来的 0.5 倍，并使用最近邻插值法（cv::INTER_NEAREST）来进行插值。这种方法不考虑像素之间的关系，而是将目标像素的值直接设为最近邻像素的值。
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        return new_frame;
    }

} // namespace myslam