#ifndef __MAP_MAPPING_H_
#define __MAP_MAPPING_H_
#include "map_point.h"
#include "frame.h"

namespace VISUAL_MAPPING {

    class Mapping {
    public:
        Mapping() = default;
        ~Mapping() = default;
        void construct_initial_map(std::vector<std::shared_ptr<Frame>>& frames);
        Map map;
        int map_point_cnt = 0;
    };
}



#endif //__MAP_MAPPING_H_
