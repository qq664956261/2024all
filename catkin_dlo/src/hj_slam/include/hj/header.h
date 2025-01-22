
// created by: zhangcheng
#ifndef HEADER_HH
#define HEADER_HH

#include <iostream>

class Header {
public:
    Header() =default;
    std::string frame_id;
    double stamp{0.0};
    unsigned int seq{0};
};

#endif //HEADER_HH
