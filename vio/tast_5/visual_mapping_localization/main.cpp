#include <iostream>
#include <thread>
#include "frame.h"
#include "mapping.h"
#include "fstream"
#include "triangulation.h"
#include "bundle_adjustment.h"
#include "visualization.h"
#include "matcher.h"
#include "detection.h"
#include "camera.h"
#include "map_save.h"

// read yaml opencv
#include "opencv2/core.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/opencv.hpp"

using namespace VISUAL_MAPPING;

std::vector<std::pair<std::string, std::string>> read_img_path(const std::string &path1, const std::string &path2, const std::string &list, std::vector<Eigen::Matrix4d> &T)
{
    std::ifstream file;
    file.open(list);
    std::vector<std::pair<std::string, std::string>> img_list;
    std::string line;
    while (std::getline(file, line, '.'))
    {
        img_list.emplace_back(path1 + line + ".png", path2 + line + ".png");
        Eigen::Matrix4d T_;
        Eigen::Vector3d t;
        Eigen::Quaterniond q;
        for (int i = 0; i < 8; i++)
        {
            std::getline(file, line, ' ');
            if (i == 1)
            {
                t(0) = std::stod(line);
                std::cout << "1:" << t(0) << std::endl;
            }
            else if (i == 2)
            {
                t(1) = std::stod(line);
            }
            else if (i == 3)
            {
                t(2) = std::stod(line);
            }
            else if (i == 4)
            {
                q.x() = std::stod(line);
            }
            else if (i == 5)
            {
                q.y() = std::stod(line);
            }
            else if (i == 6)
            {
                q.z() = std::stod(line);
            }
            else if (i == 7)
            {
                q.w() = std::stod(line);
                std::cout << "7:" << q.w() << std::endl;
            }
        }
        std::getline(file, line, '\n');
        // q.w() = std::stod(line);
        T_.block<3, 3>(0, 0) = q.toRotationMatrix();
        T_.block<3, 1>(0, 3) = t;
        T_.row(3) << 0, 0, 0, 1;
        T.push_back(T_);
    }
    return img_list;
}

void read_cam_params(const std::string &path, Camera &cam1, Camera &cam2, Eigen::Matrix4d &T12)
{
    // yaml file
    // cv::FileStorage fs(path, cv::FileStorage::READ);
    bool use_euroc = true;
    if (use_euroc)
    {
        cam1.setModelType(PINHOLE);
        cam2.setModelType(PINHOLE);
        auto param1 = new Camera::PinholeParams(458.654, 457.296, 367.215, 248.375, -0.28340811, 0.07395907, 0, 0.00019359, 1.76187114e-05);
        auto param2 = new Camera::PinholeParams(457.587, 456.134, 379.999, 255.238, -0.28368365, 0.07451284, 0, -0.00010473, -3.55590700e-05);
        cam1.setPinholeParams(*param1);
        cam2.setPinholeParams(*param2);
        Eigen::Matrix4d Tw1;
        Tw1.setIdentity();
        Tw1 << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
            0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
            -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
            0.0, 0.0, 0.0, 1.0;
        Eigen::Matrix4d Tw2;
        Tw2.setIdentity();
        Tw2 << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0;
        T12 = Tw1.inverse() * Tw2;
        std::cout << "T12:" << T12 << std::endl;
    }
    else
    {
        // if (!fs.isOpened())
        // {
        //     std::cerr << "Failed to open " << path << std::endl;
        //     return;
        // }
        // std::string type = fs["Camera.type"];
        // if (type == "pinhole")
        // {
        //     double fx = fs["fx"];
        // }
        // else if (type == "KannalaBrandt8")
        // {
        //     cam1.setModelType(KANNALA_BRANDT8);
        //     cam2.setModelType(KANNALA_BRANDT8);
        //     double fx = fs["Camera1.fx"];
        //     double fy = fs["Camera1.fy"];
        //     double cx = fs["Camera1.cx"];
        //     double cy = fs["Camera1.cy"];
        //     double k1 = fs["Camera1.k1"];
        //     double k2 = fs["Camera1.k2"];
        //     double k3 = fs["Camera1.k3"];
        //     double k4 = fs["Camera1.k4"];
        //     auto param2 = new Camera::KannalaBrandt8Params(fx, fy, cx, cy, k1, k2, k3, k4);
        //     cam1.setKannalaBrandt8Params(*param2);
        //     fx = fs["Camera2.fx"];
        //     fy = fs["Camera2.fy"];
        //     cx = fs["Camera2.cx"];
        //     cy = fs["Camera2.cy"];
        //     k1 = fs["Camera2.k1"];
        //     k2 = fs["Camera2.k2"];
        //     k3 = fs["Camera2.k3"];
        //     k4 = fs["Camera2.k4"];
        //     param2 = new Camera::KannalaBrandt8Params(fx, fy, cx, cy, k1, k2, k3, k4);
        //     cam2.setKannalaBrandt8Params(*param2);
        // }
        // cv::Mat T12_;
        // fs["Stereo.T_c1_c2"] >> T12_;

        // for (int i = 0; i < 4; i++)
        // {
        //     for (int j = 0; j < 4; j++)
        //     {
        //         float a = T12_.at<float>(i, j);
        //         T12(i, j) = a;
        //     }
        // }
    }
    // fs.release();
}

int main()
{
    // 1. read images list and camera parameters
    std::string img_path = "/home/zc/data/vio/MH_05_difficult/mav0/cam0/data/";
    std::string img_path2 = "/home/zc/data/vio/MH_05_difficult/mav0/cam1/data/";
    std::string img_list_path = "/home/zc/code/vio/orbslam+superpoint/ORB_SLAM3-master/f_MH05.txt";
    std::vector<Eigen::Matrix4d> T;
    auto img_list = read_img_path(img_path, img_path2, img_list_path, T);

    std::string cam_param_path = "/home/vio/Code/VIO/visual_localization/ORB_SLAM3_localization/Examples/Stereo/4seasons.yaml";
    Camera cam1, cam2;
    Eigen::Matrix4d T12;
    read_cam_params(cam_param_path, cam1, cam2, T12);

    std::string feature_list_path = "/home/vio/Dataset/features.yaml";
    std::vector<std::string> feature_list;

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    // 2. create frame
    //    img_list.resize(20);
    std::vector<std::shared_ptr<Frame>> frames;
    std::shared_ptr<FeatureDetection> detection =
        std::make_shared<FeatureDetection>(D2Net, "/home/zc/code/vio/tast_3/OpenVINO/learned_features_inference-openvino/weight/sp",
                                           8, 500, 512, 512);
    img_list.erase(img_list.begin(), img_list.begin() + 157);
    T.erase(T.begin(), T.begin() + 157);
    Eigen::Matrix4d init_T = *T.begin();
    for (auto &i : T)
    {
        i = init_T.inverse() * i;
    }
    for (int i = 0; i < img_list.size(); i++)
    {
        cv::Mat img1 = cv::imread(img_list[i].first, cv::IMREAD_GRAYSCALE);
        cv::Mat img2 = cv::imread(img_list[i].second, cv::IMREAD_GRAYSCALE);
        cv::resize(img1, img1, cv::Size(512, 512));
        cv::resize(img2, img2, cv::Size(512, 512));
        clahe->apply(img1, img1);
        clahe->apply(img2, img2);
        cv::cvtColor(img1, img1, cv::COLOR_GRAY2BGR);
        cv::cvtColor(img2, img2, cv::COLOR_GRAY2BGR);

        auto frame = std::make_shared<Frame>(i, detection, T[i], img1, img2, &cam1, &cam2, T12);
        std::cout << "frame " << i << std::endl;
        frames.push_back(frame);
        //        cv::imshow("show", img1);
        //        cv::waitKey(0);
    }

    // 3. init mapping and refine
    Mapping mapping;
    mapping.construct_initial_map(frames);
    for (int i = 0; i < 2; i++)
    {
        std::cout << "start refine map " << i << std::endl;
        mapping.refine_map();
    }

    // 4. save map
    MapSaver mapSaver;
    mapSaver.save_map("map_d2net.txt", frames, mapping.map);

    //    MapSaver mapSaver2;
    //    Mapping mapping2;
    //    std::vector<std::shared_ptr<Frame>> frames2;
    //    mapSaver2.load_map("map.txt", frames2, mapping2.map);

    // 5. visualization thread
    Visualization vis;
    std::thread vis_thread(Visualization::run, &vis, std::ref(mapping.map));
    vis_thread.join();
}
