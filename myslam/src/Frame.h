#pragma once

#include "common_include.h"
#include "Camera.h"
namespace myslam {
	class Frame
	{
	public:
		typedef std::shared_ptr<Frame> Ptr;
		unsigned long id_;
		double time_stamp_;
		SE3d T_c_w_;
		Camera::Ptr camera_;
		Mat color_, depth_;

	public:
		Frame();
		Frame(long id, double time_stamp = 0, SE3d T_c_w = SE3d(), Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
		~Frame();

		// factory function
		static Frame::Ptr createFrame();

		// find the depth in depth map
		double findDepth(const cv::KeyPoint& kp);

		// get camera center
		Vector3d getCameraCenter() const;

		// check if a point is in this frame
		bool isInFrame(const Vector3d& pt_world);

	};

}

