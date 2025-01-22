#include "scanmatcher.h"
#include <cstring>
#include <limits>
#include <list>
#include <iostream>
#include "gridlinetraversal.h"

namespace HJ_2dLaserMapping
{

	using namespace std;

	ScanMatcher::ScanMatcher()
	{
		m_linePoints = new IntPoint[20000];
	}

	ScanMatcher::~ScanMatcher()
	{
		delete [] m_linePoints;
	}

	// void ScanMatcher::registerScan(ScanMatcherMap &map, const hj_slam::Lds &msg)
	// {
	// 	// 为activeArea里面的没有分配内存的区域分配内存
	// 	// map.storage().allocActiveArea();
	// 	/*把激光雷达的位置转换到地图坐标系*/
	// 	OrientedPoint lp(msg.ldsPose.pose.x, msg.ldsPose.pose.y, msg.ldsPose.pose.theta);
	// 	// 激光雷达在栅格地图上的栅格坐标p0
	// 	IntPoint p0 = map.world2map(lp);
	// 	for (int i = 0; i < msg.points.size(); ++i)
	// 	{
	// 		/*去除非法的激光束*/
	// 		double d = msg.points[i].rho;
	// 		if (d > m_laserMaxRange || d == 0.0 || isnan(d))
	// 			continue;
	// 		// if(msg.points[i].theta > 1.05 && (msg.points[i].theta < 5.23))
	// 		// 		continue;
	// 		if (d > m_usableRange)
	// 			d = m_usableRange;
	// 		/*被该激光束击中的点的栅格坐标p1*/
	// 		Point phit = lp + Point(d * cos(lp.theta + msg.points[i].theta), d * sin(lp.theta + msg.points[i].theta));
	// 		IntPoint p1 = map.world2map(phit);
	// 		/*bresenham画线算法来计算 激光位置和被激光击中的位置之间的空闲位置*/
	// 		GridLineTraversalLine line;
	// 		line.points = m_linePoints;
	// 		GridLineTraversal::gridLine(p0, p1, &line);
	// 		/*更新空闲位置*/
	// 		for (int i = 0; i < line.num_points - 1; i++)
	// 		{ // 未击中，就不记录击中的位置了，所以传入参数Point(0,0)
	// 			map.cell(line.points[i]).update(false, Point(0, 0));
	// 		}
	// 		/*更新被击中的位置　只有小于m_usableRange的栅格来用来标记障碍物*/
	// 		if (d < m_usableRange)
	// 		{
	// 			// 击中，记录击中的位置，所以传入参数phit，击中点的物理位置
	// 			map.cell(p1).update(true, phit);
	// 		}
	// 	}
	// }

	void ScanMatcher::registerScan2(ScanMatcherMap &map, const hjSlam_2d::Lds &msg)
	{
		/*把激光雷达的位置转换到地图坐标系*/
		OrientedPoint lp(msg.ldsPose.pose.x, msg.ldsPose.pose.y, msg.ldsPose.pose.theta);

		// 激光雷达在栅格地图上的栅格坐标p0
		IntPoint p0 = map.world2map(lp);

		for (int i = 0; i < msg.points.size(); ++i)
		{
			Point phit(msg.points[i].x, msg.points[i].y);
			IntPoint p1 = map.world2map(phit);
			Point p0phit = phit - Point(msg.ldsPose.pose.x, msg.ldsPose.pose.y);
			double d = euclidianDist(p0phit, Point());
			if (d > m_laserMaxRange || d == 0.0 || isnan(d))
				continue;

			if (d > m_usableRange){
				phit = Point(msg.ldsPose.pose.x, msg.ldsPose.pose.y) + Point(m_usableRange / d * p0phit.x, m_usableRange / d * p0phit.y);
				p1 = map.world2map(phit);
			}

			/*bresenham画线算法来计算 激光位置和被激光击中的位置之间的空闲位置*/
			GridLineTraversalLine line;
			line.points = m_linePoints;
			GridLineTraversal::gridLine(p0, p1, &line);

			/*更新空闲位置*/
			for (int i = 0; i < line.num_points - 1; i++)
			{ // 未击中，就不记录击中的位置了，所以传入参数Point(0,0)
				map.cell(line.points[i]).update(false, Point(0, 0));
			}

			/*更新被击中的位置　只有小于m_usableRange的栅格来用来标记障碍物*/
			if (d < m_usableRange)
			{
				// 击中，记录击中的位置，所以传入参数phit，击中点的物理位置
				map.cell(p1).update(true, phit);
			}
			
		}
	}
};
