#include "fast_gicp.hpp"
#include "fast_gicp_impl.hpp"

template class fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICP<pcl::PointXYZI, pcl::PointXYZI>;
