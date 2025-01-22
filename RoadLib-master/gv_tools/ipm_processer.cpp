#include "ipm_processer.h"
#include <camodocal/camera_models/CataCamera.h>
#include <camodocal/camera_models/EquidistantCamera.h>
#include <camodocal/camera_models/PinholeCamera.h>
#include <camodocal/camera_models/PinholeFullCamera.h>
#include <camodocal/camera_models/ScaramuzzaCamera.h>

//#define POINTWISE_MAPPING
namespace gv
{
	IPMProcesser::IPMProcesser(CameraConfig conf, IPMType ipm_type) : _config(conf), _ipm_type(ipm_type) {
#ifdef POINTWISE_MAPPING
		_IPM_mapx_array = new float[_config.IPM_HEIGHT*_config.IPM_WIDTH];
		_IPM_mapy_array = new float[_config.IPM_HEIGHT*_config.IPM_WIDTH];

		memset(_IPM_mapx_array, 0.0, _config.IPM_HEIGHT*_config.IPM_WIDTH * sizeof(float));
		memset(_IPM_mapy_array, 0.0, _config.IPM_HEIGHT*_config.IPM_WIDTH * sizeof(float));

		_IPM_mapx = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1);
		_IPM_mapy = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1);
#endif

		// config.camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile("H:/2022_10_12/data_20221012133020/config/cam0_pinhole.yaml");
		if (_config.camera->modelType() == camodocal::Camera::PINHOLE)
		{
			_h = _config.camera->imageHeight();
			_w = _config.camera->imageWidth();
			_fx = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().fx();
			_fy = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().fy();
			_cx = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().cx();
			_cy = std::dynamic_pointer_cast<camodocal::PinholeCamera>(_config.camera)->getParameters().cy();
		}
		else if (_config.camera->modelType() == camodocal::Camera::PINHOLE_FULL
			|| _config.camera->modelType() == camodocal::Camera::KANNALA_BRANDT
			|| _config.camera->modelType() == camodocal::Camera::SCARAMUZZA
			|| _config.camera->modelType() == camodocal::Camera::MEI)
		{
			_h = _config.camera->imageHeight();
			_w = _config.camera->imageWidth();
			Eigen::Vector2d p0(0, 0);
			Eigen::Vector2d p1(_w - 1, _h - 1);
			Eigen::Vector3d p0_, p1_;
			_config.camera->liftProjective(p0, p0_);
			_config.camera->liftProjective(p1, p1_);
			p0_ /= p0_(2);
			p1_ /= p1_(2);
			_fx = (_w - 1) / (p1_(0) - p0_(0));
			_fy = (_h - 1) / (p1_(1) - p0_(1));
			_cx = -_fx * p0_(0);
			_cy = -_fy * p0_(1);
		}
		else
		{
			throw std::exception();
		}
		_config.camera->initUndistortRectifyMap(_undist_M0, _undist_M1, _fx, _fy, cv::Size(_w, _h), _cx, _cy);
		this->updateCameraGroundGeometry(_config.cg);
	}
	IPMProcesser::~IPMProcesser()
	{
		if (_IPM_mapx_array) delete[] _IPM_mapx_array;
		if (_IPM_mapy_array) delete[] _IPM_mapy_array;
	}
	//updateCameraGroundGeometry：更新相机与地面的几何关系，并调用 updateIPMMap 更新 IPM 映射。
    int IPMProcesser::updateCameraGroundGeometry(CameraGroundGeometry cg)
    {
		_Rc0c1 = cg.getR();
		_d = cg.getH();
		return updateIPMMap();
    }
	// 估计地面区域在原始图像中的大致位置，并生成一个对应这个区域的掩膜（mask）。
    cv::Mat IPMProcesser::guessGroundROI()
    {
		cv::Point p0 = IPM2Perspective(cv::Point2f(0,0));
		cv::Point p1 = IPM2Perspective(cv::Point2f(_config.IPM_WIDTH-1,0));
		cv::Point p2 = IPM2Perspective(cv::Point2f(_config.IPM_WIDTH-1,_config.IPM_HEIGHT/2));
		cv::Point p3 = IPM2Perspective(cv::Point2f(0,_config.IPM_HEIGHT/2));
		std::vector<cv::Point> pts({p0,p1,p2,p3});
		cv::Mat mask = cv::Mat(_config.camera->imageHeight(), _config.camera->imageWidth(), CV_8UC1);
		mask.setTo(0);
		cv::fillConvexPoly(mask,pts,255);
        return mask;
    }
// 该函数的功能是更新像逆透视映射（Inverse Perspective Mapping, IPM）地图，用于将摄像机捕获的图像变换为鸟瞰图（顶视图）。
    int IPMProcesser::updateIPMMap()
    {
#ifdef POINTWISE_MAPPING
		double c1_p_c1f_array[3];
		double R_c0c1_array[3][3] = { {_Rc0c1(0, 0),_Rc0c1(0, 1),_Rc0c1(0, 2)},
									  {_Rc0c1(1, 0),_Rc0c1(1, 1),_Rc0c1(1, 2)},
									  {_Rc0c1(2, 0),_Rc0c1(2, 1),_Rc0c1(2, 2)} };
		double xy1_y_array[3];
		if (_ipm_type == IPMType::NORMAL)
		{
			for (int i = 0; i < _config.IPM_HEIGHT; i++)
			{
				for (int j = 0; j < _config.IPM_WIDTH; j++)
				{
					c1_p_c1f_array[0] = (j - _config.IPM_WIDTH / 2) * _config.IPM_RESO;
					c1_p_c1f_array[1] = _d;
					c1_p_c1f_array[2] =(_config.IPM_HEIGHT - i - 1) * _config.IPM_RESO;
					xy1_y_array[0] = (R_c0c1_array[0][0] * c1_p_c1f_array[0] + R_c0c1_array[0][1] * c1_p_c1f_array[1] + R_c0c1_array[0][2] * c1_p_c1f_array[2]);
					xy1_y_array[1] = (R_c0c1_array[1][0] * c1_p_c1f_array[0] + R_c0c1_array[1][1] * c1_p_c1f_array[1] + R_c0c1_array[1][2] * c1_p_c1f_array[2]);
					xy1_y_array[2] = (R_c0c1_array[2][0] * c1_p_c1f_array[0] + R_c0c1_array[2][1] * c1_p_c1f_array[1] + R_c0c1_array[2][2] * c1_p_c1f_array[2]);
					_IPM_mapx_array[i*_config.IPM_WIDTH + j] = _fx * xy1_y_array[0] / xy1_y_array[2] + _cx;
					_IPM_mapy_array[i*_config.IPM_WIDTH + j] = _fy * xy1_y_array[1] / xy1_y_array[2] + _cy;
				}
			}
			_IPM_mapx = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1, _IPM_mapx_array);
			_IPM_mapy = cv::Mat(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_32FC1, _IPM_mapy_array);
			return 0;
		}
		else
		{
			return -1;
		}

#else
		// 定义内参矩阵K和IPM的逆变换矩阵K_IPM_inv
		// K_IPM_inv是一个根据IPM图像分辨率和相机高度_d计算地面上点到像素坐标的逆变换矩阵
		Eigen::Matrix3d K, K_IPM_inv;
		K << _fx, 0, _cx,
			0, _fy, _cy,
			0, 0, 1;
		if (_ipm_type == IPMType::NORMAL)
		{
			K_IPM_inv << _config.IPM_RESO / _d, 0, (-_config.IPM_WIDTH / 2)*_config.IPM_RESO / _d,
				            0, 0, 1,
				         0, -_config.IPM_RESO / _d, (_config.IPM_HEIGHT - 1)*_config.IPM_RESO / _d;
			//  这个公式用于计算逆透视映射（Inverse Perspective Mapping, IPM）的变换矩阵T_IPM，它将从相机捕获的图像映射到一个顶视图（鸟瞰图）
			// 			组成部分
			// **K**：相机的内参矩阵。它包含了相机焦距（_fx, _fy）和光心（_cx, _cy）的信息，用于将三维空间中的点（用相机坐标系表示）投影到二维图像平面上。
			// **_Rc0c1**：从相机坐标系到地面坐标系（或相机初始位置到当前位置）的旋转矩阵。这个旋转矩阵描述了相机相对于地面（或其初始位置）的旋转。
			// **_d**：相机距离地面的高度。这个距离乘以上述旋转矩阵，可以调整因相机高度引起的尺度变化。
			// **K_IPM_inv**：IPM图像的逆内参矩阵。与K描述了如何将三维点投影到二维图像上不同，K_IPM_inv用于将IPM图像中的像素坐标反向映射回对应的三维点。
			// 物理意义
			// 这个公式的核心是构建一个变换矩阵，它可以一次性完成以下几个步骤：
			// 反向映射：首先，使用K_IPM_inv，将IPM图像中的像素坐标反向映射到一个处于地面平面的三维空间点上。这个点是在地面坐标系中表示的。
			// 逆旋转：然后利用_Rc0c1的逆矩阵（也就是说，将_Rc0c1乘以_d），对这个点进行逆向旋转，将它从地面坐标系变换回相机坐标系（或将它从相机当前位置变换回初始位置）。
			// 逆内参变换：最终，利用K的逆矩阵（由于K在公式中先被使用，因此在整个变换中的作用相当于它的逆），将上述变换后的三维点投影回二维图像平面。
			// 这个变换矩阵T_IPM的逆（也就是T_IPM.inverse()部分）, 实际上是用来直接将图像上的点映射到IPM视图对应的点。由于该公式目的是计算正向变换（从原图到IPM图），但直接计算的是反向过程，因此需要取逆来使用。
			Eigen::Matrix3d T_IPM = (K * _Rc0c1 * _d * K_IPM_inv).inverse();
			cv::eigen2cv(T_IPM, _IPM_transform);
			_IPM_transform.convertTo(_IPM_transform, CV_32F);
			return 0;
		}
		else
		{
			return -1;
		}
#endif
	}
	//去畸变
	cv::Mat IPMProcesser::genUndistortedImage(cv::Mat mm) const
	{
		cv::Mat mm_undist;
		cv::remap(mm, mm_undist, _undist_M0, _undist_M1, cv::INTER_NEAREST);
		return mm_undist;
	}
	//用于图像处理中的逆透视映射（IPM）和透视图像坐标转换
	cv::Mat IPMProcesser::genIPM(cv::Mat mm, bool need_undistort, cv::Mat mm_undist)  const
	{
		if (need_undistort)
		{
			cv::remap(mm, mm_undist, _undist_M0, _undist_M1, cv::INTER_NEAREST);
		}
		else
		{
			mm_undist = mm;
		}
		cv::Mat m_ipm(_config.IPM_HEIGHT, _config.IPM_WIDTH, CV_8U);
#ifdef POINTWISE_MAPPING
		cv::remap(mm_undist, m_ipm, _IPM_mapx, _IPM_mapy, cv::INTER_NEAREST);
		cv::remap(mm_undist, m_ipm, _IPM_mapx, _IPM_mapy, cv::INTER_LINEAR);
#else
		cv::warpPerspective(mm_undist, m_ipm, _IPM_transform, m_ipm.size(), cv::INTER_NEAREST);
#endif
		return m_ipm;
	}
	//用于返回相机内参
	cv::Vec4d IPMProcesser::getUndistortedIntrinsics()
	{
		return cv::Vec4d(_fx, _fy, _cx, _cy);
	}

	// 这段代码定义了函数IPMProcesser::IPM2Perspective，它的主要目的是将逆透视映射（IPM）中的一个点转换回原始图像（透视图）中的坐标。
	// 这个函数较为复杂，融合了IPM图像坐标到透视图像坐标的几何变换
	cv::Point2f IPMProcesser::IPM2Perspective(cv::Point2f p, CameraGroundGeometry* cg_temp)  const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Eigen::Vector3d((p.x - _config.IPM_WIDTH / 2) * _config.IPM_RESO,
				d,
				(_config.IPM_HEIGHT - p.y - 1) * _config.IPM_RESO);
			Eigen::Vector3d pc0f = Rc0c1 * pc1f;
			Eigen::Vector3d uv_c0 = Eigen::Vector3d(pc0f(0) / pc0f(2) *_fx + _cx, pc0f(1) / pc0f(2)*_fy + _cy, 1);
			return cv::Point2f(uv_c0(0), uv_c0(1));
		}
		else
		{
			throw std::exception();
		}
	}
	//将原始透视图像中的一个点转换为逆透视映射（IPM）图像中的坐标
	cv::Point2f IPMProcesser::Perspective2IPM(cv::Point2f p, CameraGroundGeometry* cg_temp)  const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		double x = (p.x - _cx) / _fx;
		double y = (p.y - _cy) / _fy;

		cv::Point2f pipm;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d xyzc1f = (Rc0c1.transpose()*Eigen::Vector3d(x, y, 1));
			double depth = d / xyzc1f(1);
			//std::cerr << depth << std::endl;
			Eigen::Vector3d pc1f = xyzc1f * depth;
			//std::cerr << pc1f.transpose() << std::endl;
			pipm = cv::Point2f(pc1f.x() / _config.IPM_RESO + _config.IPM_WIDTH / 2,
				              -pc1f.z() / _config.IPM_RESO + _config.IPM_HEIGHT - 1);
			return  pipm;
		}
		else
		{
			throw std::exception();
		}
	}
	//将原始透视图像中的一个点转换为相对于摄像机坐标系中的3D度量坐标
	Eigen::Vector3d IPMProcesser::Perspective2Metric(cv::Point2f p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		double x = (p.x - _cx) / _fx;
		double y = (p.y - _cy) / _fy;

		Eigen::Vector3d pc0f;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d xyzc1f = (Rc0c1.transpose()*Eigen::Vector3d(x, y, 1));
			double depth = d / xyzc1f(1);
			//std::cerr << depth << std::endl;
			Eigen::Vector3d pc1f = xyzc1f * depth;
			pc0f = Rc0c1 * pc1f;
			return  pc0f;
		}
		else
		{
			throw std::exception();
		}
	}
	//将归一化透视图像中的一个点转换为相对于摄像机坐标系中的3D度量坐标
	Eigen::Vector3d IPMProcesser::NormalizedPerspective2Metric(Eigen::Vector3d p, CameraGroundGeometry* cg_temp) const
	{
		return Perspective2Metric(cv::Point2f(p.x()*_fx + _cx, p.y()*_fy + _cy), cg_temp);
	}

	Eigen::Vector3d IPMProcesser::IPM2Metric(cv::Point2f p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Eigen::Vector3d((p.x - _config.IPM_WIDTH / 2) * _config.IPM_RESO,
				                                   d,
				                                   (_config.IPM_HEIGHT - p.y - 1) * _config.IPM_RESO);
			Eigen::Vector3d pc0f = Rc0c1 * pc1f;
			return pc0f;
		}
		else
		{
			throw std::exception();
		}
	}
	//将相机坐标系中的3D度量坐标转换为逆透视映射（IPM）图像中的坐标
	cv::Point2f IPMProcesser::Metric2IPM(Eigen::Vector3d p, CameraGroundGeometry* cg_temp) const
	{
		Eigen::Matrix3d Rc0c1 = _Rc0c1;
		double d = _d;
		if(cg_temp!=nullptr)
		{
			Rc0c1 = cg_temp->getR();
			d = cg_temp->getH();
		}

		cv::Point2f uv;
		if (_ipm_type == IPMType::NORMAL)
		{
			Eigen::Vector3d pc1f = Rc0c1.transpose() * p;
			uv.x = pc1f(0) / _config.IPM_RESO + _config.IPM_WIDTH / 2;
			uv.y = _config.IPM_HEIGHT - 1 - pc1f(2) / _config.IPM_RESO;
			return uv;
		}
		else
		{
			throw std::exception();
		}
	}
}

	