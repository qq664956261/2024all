#include "fast_gicp_st.hpp"
#include "fast_gicp_st_impl.hpp"

template class fast_gicp::FastGICPSingleThread<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastGICPSingleThread<pcl::PointXYZI, pcl::PointXYZI>;
