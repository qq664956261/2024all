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
    }
}
