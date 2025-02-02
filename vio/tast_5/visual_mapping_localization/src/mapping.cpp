#include "mapping.h"
#include "matcher.h"
#include "triangulation.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "bundle_adjustment.h"


namespace VISUAL_MAPPING {

    /**
     * @brief Construct the initial map
     *  1. Construct the initial map points from stereo images
     *  2. Triangulate the map points
     *  3. Link the map points to other frames
     * @param frames
     */
    void Mapping::construct_initial_map(std::vector<std::shared_ptr<Frame>>& frames) {
        map.frames_ = frames;
        int frame_cnt = 0; // frame id

        std::cout << "step 1: Constructing initial map points for stereo !" << std::endl;
        for (auto& frame : frames) {// for each frame in the sequence
            for (int i = 0; i < frame->get_features_uv().size(); i++) {
                Eigen::Vector2d uv = frame->get_features_uv()[i];
                double depth = frame->get_features_depth()[i];
                double cov = frame->features_depth_cov[i];
                std::shared_ptr<MapPoint> mp_ptr = frame->map_points[i];
                // if the depth is valid and the map point is not created
                if (depth > 0.01 && mp_ptr == nullptr) {
                    // create a new map point and add it to the map
                    Eigen::Vector3d x3D = frame->get_camera()->pixel2camera(uv, depth);
                    x3D = frame->get_R() * x3D + frame->get_t();
                    std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>(uv, x3D, cov, frame->id, i, map_point_cnt);
                    frame->map_points[i] = map_point;
                    map.add_map_point(map_point);
                    map_point_cnt++;
                }
            }
            frame_cnt ++;
        }
        std::cout << "Initial constructed " << map_point_cnt << " map points" << std::endl;

        std::cout << "step 2: Find the connected frames !" << std::endl;
        std::cout << "step 2.1: Find the connected frames by re-projecting the map points to other frames !" << std::endl;
        std::vector<std::vector<std::shared_ptr<Frame>>> connected_frames;
        connected_frames.resize(frames.size());
        for (int i = 0; i < frames.size(); i++) {
            std::shared_ptr<Frame> frame0 = frames[i];
            for (int j = i + 1; j < frames.size(); j ++) {
                std::shared_ptr<Frame> frame1 = frames[j];
                double dist = (frame0->get_t() - frame1->get_t()).norm();
                if (dist > 2) {
                    continue;
                }
                int project_success = 0;
                for (const auto& mp : frame0->map_points) {
                    if (mp != nullptr) {
                        Eigen::Vector3d P = mp->x3D;
                        Eigen::Vector3d P_ = frame1->get_R().transpose() * (P - frame1->get_t());
                        Eigen::Vector2d uv = frame1->get_camera()->project(P_);
                        if (frame1->in_image(uv)) {
                            project_success++;
                        }
                        if (project_success > 10) {
                            connected_frames[i].push_back(frame1);
                            break;
                        }
                    }
                }
            }
        }

        std::cout<<"step 2.2:  Find map point connections: "<<std::endl;
        Matcher matcher;

        // init the connected frames
        map.connected_frames.resize(frames.size());
        for (int i = 0;i < frames.size(); i++) {
            map.connected_frames[i].emplace_back(frames[i]);
        }

        // match the map points
        for (int i = 0;i < frames.size(); i++) {
            std::shared_ptr<Frame> frame0 = frames[i];
            std::vector<std::shared_ptr<Frame>> frames1 = connected_frames[i];

            for (const auto& frame1 : frames1) {
                std::vector<std::pair<int, int>> matches = matcher.match_re_projective(frame0, frame1);

                if (matches.size() > 30){// connect frames
                    frame0->add_linked_frame(frame1);
                    frame1->add_linked_frame(frame0);
                    map.connected_frames[i].emplace_back(frame1);
                    map.connected_frames[frame1->id].emplace_back(frame0);
                }

                for (const auto& match : matches) {// connect map points
                    const auto& mp0 = frame0->map_points[match.first];
                    const auto& mp1 = frame1->map_points[match.second];
                    mp0->add_measurement(frame1, match.second);
                    if (mp1 != nullptr) {
                        mp1->add_measurement(frame0, match.first);
                    }
                }
            }
        }
    }

    void Mapping::refine_map() {
        std::cout << "Refine map step 1: Update all map points !" << std::endl;
        std::cout<<" map points size: "<<map.map_points_count<<std::endl;
        BundleAdjustment ba;
        for (const auto& frame : map.frames_) {
            for (int mp_id = 0; mp_id < frame->map_points.size(); mp_id ++) {
                const auto& mp = frame->map_points[mp_id];
                if (mp != nullptr) {
                    // find all connected map points
                    std::vector<std::shared_ptr<Frame>> c_frames;
                    std::vector<int> c_ids;
                    std::unordered_map<std::shared_ptr<Frame>, int> connected_mps;
                    for (int i = 0; i < mp->frames.size(); i++) {
                        if (connected_mps.find(mp->frames[i]) == connected_mps.end()) {
                            connected_mps[mp->frames[i]] = mp->frame_feature_ids[i];
                            std::shared_ptr<MapPoint> c_mp = mp->frames[i]->map_points[mp->frame_feature_ids[i]];
                            if (c_mp != nullptr) {
                                for (int j = 0; j < c_mp->frames.size(); j++) {
                                    if (connected_mps.find(c_mp->frames[j]) == connected_mps.end()) {
                                        connected_mps[c_mp->frames[j]] = c_mp->frame_feature_ids[j];
                                    }
                                }
                            }
                        }
                    }
                    // add current frame
                    c_frames.push_back(frame);
                    c_ids.push_back(mp_id);

                    for (const auto& kv : connected_mps) {
                        if (kv.first == frame) {
                            continue;
                        }
                        c_frames.push_back(kv.first);
                        c_ids.push_back(kv.second);
                    }
                    // optimize the map point position
                    if (c_frames.size() > 1) {
                        // visualize the connected frames and project the map point
//                        {
//                            Eigen::Vector3d xyz = mp->x3D;
//                            for (int i = 0; i < c_frames.size(); i++){
//                                cv::Mat show = c_frames[i]->image.clone();
//                                if (show.channels() == 1) {
//                                    cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
//                                }
//                                cv::circle(show, cv::Point((int)c_frames[i]->get_features_uv()[c_ids[i]][0], (int)c_frames[i]->get_features_uv()[c_ids[i]][1]), 4, cv::Scalar(0, 0, 255), 2);
//                                Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                                cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 2, cv::Scalar(0, 255, 0), 2);
//                                cv::imwrite("show" + std::to_string(i) + ".png", show);
//                            }
//                        }
//                        double error = 0, update_error = 0;
                        Eigen::Vector3d xyz = mp->x3D;
//                        for (int i = 0; i < c_frames.size(); i++){
//                            Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                            error += (uv - c_frames[i]->get_features_uv()[c_ids[i]]).norm();
//                        }
                        std::vector<int> status(c_frames.size(), 1);
                        ba.optimize_structure(c_frames, c_ids, xyz, status);
                        // update the map point position
                        mp->x3D = xyz;
//                        for (int i = 0; i < c_frames.size(); i++){
//                            Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                            double e = (uv - c_frames[i]->get_features_uv()[c_ids[i]]).norm();
//                            update_error += e;
//                        }
//                        std::cout << "error: " << error << " -> " << update_error << std::endl;

                        // visualize the connected frames and project the map point
//                        {
//                            for (int i = 0; i < c_frames.size(); i++){
//                                cv::Mat show = c_frames[i]->image.clone();
//                                if (show.channels() == 1) {
//                                    cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
//                                }
//                                cv::circle(show, cv::Point((int)c_frames[i]->get_features_uv()[c_ids[i]][0], (int)c_frames[i]->get_features_uv()[c_ids[i]][1]), 4, cv::Scalar(0, 0, 255), 2);
//                                Eigen::Vector2d uv = c_frames[i]->get_camera()->project(c_frames[i]->get_R().transpose() * (xyz - c_frames[i]->get_t()));
//                                cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 2, cv::Scalar(0, 255, 0), 2);
//                                cv::imwrite("u_show" + std::to_string(i) + ".png", show);
//                            }
//                        }

                        // fuse the map points
                        for (int i = 0; i < c_frames.size(); i++) {
                            if (status[i] == 1 && c_frames[i]->map_points[c_ids[i]] != mp) {
                                if (c_frames[i]->map_points[c_ids[i]] != nullptr) {
                                    map.remove_map_point(c_frames[i]->map_points[c_ids[i]]->id);
                                }
                                const auto del_mp = c_frames[i]->map_points[c_ids[i]];
                                if (del_mp != nullptr) {
                                    for (int j = 0;j < del_mp->frame_ids.size();j ++) {
                                        int c_frame_id = del_mp->frame_ids[j];
                                        int c_feature_id = del_mp->feature_ids[j];
                                        Eigen::Vector2d c_uv = del_mp->features_uv[j];
                                        map.frames_[c_frame_id]->map_points[c_feature_id] = mp;
                                        mp->add_frame(c_frame_id, c_feature_id, 0, c_uv, mp->x3D);
                                    }
                                } else {
                                    c_frames[i]->map_points[c_ids[i]] = mp;
                                    mp->add_frame(c_frames[i]->id, c_ids[i], 0, c_frames[i]->get_features_uv()[c_ids[i]], mp->x3D);
                                }

                            }
                        }

                    }
                }
            }
        }
        std::cout<<" map points size: "<<map.map_points_count<<std::endl;

        std::cout << "Refine map step 2: update all frame pose !" << std::endl;
        for (const auto& frame : map.frames_) {
            // 1. compute the projection error
           cv::Mat show = frame->image.clone();
           if (show.channels() == 1) {
               cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
           }
           double sum_error = 0;
           for (int i = 0;i < frame->map_points.size(); i++) {
               const auto& mp = frame->map_points[i];
               if (mp != nullptr) {
                   Eigen::Vector3d P = mp->x3D;
                   Eigen::Vector3d P_ = frame->get_R().transpose() * (P - frame->get_t());
                   Eigen::Vector2d uv = frame->get_camera()->project(P_);
                   double error = (uv - frame->get_features_uv()[i]).norm();
                   sum_error += error;
                   cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 4, cv::Scalar(0, 255, 0), 2);
                   cv::circle(show, cv::Point((int)frame->get_features_uv()[i][0], (int)frame->get_features_uv()[i][1]), 2, cv::Scalar(0, 0, 255), 2);
               }
           }
           cv::imwrite("show.png", show);
           std::cout<<" frame id: "<<frame->id<<" error: "<<sum_error<<std::endl;

            // 2. pose optimization
            ba.optimize_pose(frame);

            // 3. compute the projection error after optimization
           show = frame->image.clone();
           if (show.channels() == 1) {
               cv::cvtColor(show, show, cv::COLOR_GRAY2BGR);
           }
           sum_error = 0;
           for (int i = 0;i < frame->map_points.size(); i++) {
               const auto& mp = frame->map_points[i];
               if (mp != nullptr) {
                   Eigen::Vector3d P = mp->x3D;
                   Eigen::Vector3d P_ = frame->get_R().transpose() * (P - frame->get_t());
                   Eigen::Vector2d uv = frame->get_camera()->project(P_);
                   double error = (uv - frame->get_features_uv()[i]).norm();
                   sum_error += error;
                   cv::circle(show, cv::Point((int)uv[0], (int)uv[1]), 4, cv::Scalar(0, 255, 0), 2);
                   cv::circle(show, cv::Point((int)frame->get_features_uv()[i][0], (int)frame->get_features_uv()[i][1]), 2, cv::Scalar(0, 0, 255), 2);
               }
           }
           cv::imwrite("u_show.png", show);
           std::cout<<" frame id: "<<frame->id<<" error: "<<sum_error<<std::endl;
           std::cout<<"here"<<std::endl;
        }


        std::cout << "Refine map step 3: full bundle adjustment !" << std::endl;
        ba.optimize_ba(map);

        std::cout << "Refine map step 4: add more map points !" << std::endl;
        add_points();
    }

    void Mapping::add_points() {
        Matcher matcher;
        for (auto& frame : map.frames_) {
            int frame_mps_cnt = 0;
            // 1. calculate the number of map points in the frame
            for (auto& mp : frame->map_points) {
                if (mp != nullptr) {
                    frame_mps_cnt++;
                }
            }
            // 2. if the number of map points is less than 100, add more map points
            if (frame_mps_cnt < 100) {
                std::cout<<" frame id: "<<frame->id<<" with "<<frame_mps_cnt<<" map points"<<std::endl;
                // 2.1. find the connected frames
                std::vector<std::shared_ptr<Frame>> connected_frames;
                for (const auto& f : map.connected_frames[frame->id]) {
                    connected_frames.push_back(f);
                }
                // 2.2 map point matching
                for (const auto& c_frame:connected_frames) {
                    std::vector<std::pair<int, int>> matches = matcher.match_re_projective(c_frame, frame);
                    for (const auto&match:matches) {
                        const auto& mp0 = c_frame->map_points[match.first];
                        auto& mp1 = frame->map_points[match.second];
                        if (mp0 != nullptr && mp1 == nullptr) {
                            mp1 = mp0;
                            frame_mps_cnt ++;
                        }
                    }
                }
                std::cout<<" first add map points: "<<frame_mps_cnt<<std::endl;
                // 2.3 add new map points

                if (frame_mps_cnt < 100) {

                    // 1. features wait to be matched
                    cv::Mat descriptors((int)frame->features_uv.size() - frame_mps_cnt, frame->descriptors.cols, frame->descriptors.type());
                    std::vector<int> ids;

                    ids.reserve(frame->features_uv.size() - frame_mps_cnt);
                    descriptors.reserve(frame->features_uv.size() - frame_mps_cnt);

                    for (int i = 0;i < frame->features_uv.size();i ++) {
                        if (frame->map_points[i] == nullptr) {
                            frame->descriptors.row(i).copyTo(descriptors.row((int)ids.size()));
                            ids.emplace_back(i);
                        }
                    }

                    // 2. match with all connected frames
                    std::vector<std::vector<std::pair<int, int>>> matches;

                    for (const auto&c_frame:connected_frames) {
                        matches.emplace_back(matcher.match_descriptor(descriptors, c_frame->descriptors));
                    }

                    std::vector<std::vector<int>> matched_ids;
                    std::vector<int> tmp;
                    tmp.resize(connected_frames.size(), -1);
                    matched_ids.resize((int)frame->features_uv.size() - frame_mps_cnt, tmp);

                    for (int i = 0;i < connected_frames.size();i ++) {
                        for (const auto& m:matches[i]) {
                            matched_ids[m.first][i] = m.second;
                        }
                    }

                    // 3. triangulate the map points
                    Triangulation triangulation;
                    std::vector<std::pair<double, double>> results = triangulation.multiview_triangulate(frame, ids, connected_frames, matched_ids);

                    for (int i = 0;i < results.size(); i++) {
                        const auto&result = results[i];
                        if (result.first > 0) {
                            // create a new map point and add it to the map
                            Eigen::Vector3d x3D = frame->get_camera()->pixel2camera(frame->features_uv[ids[i]], result.first);
                            x3D = frame->get_R() * x3D + frame->get_t();
                            std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>(frame->features_uv[ids[i]], x3D, result.second, frame->id, i, map_point_cnt);
                            frame->map_points[i] = map_point;
                            map.add_map_point(map_point);
                            map_point_cnt ++;
                            frame_mps_cnt ++;
                        }
                    }
                    std::cout<<"second add map points: "<<frame_mps_cnt<<std::endl;
                }
            }
        }
    }
}
