#include "smmap.h"

namespace HJ_2dLaserMapping {

//地图中使用的栅格
const PointAccumulator& PointAccumulator::Unknown()
{
	if (! unknown_ptr)
		unknown_ptr=new PointAccumulator;
	return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr = 0;

};