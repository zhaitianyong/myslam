#pragma once
#include "common_include.h"
#include "frame.h"
namespace myslam {
	class MapPoint
	{

	public:
		typedef shared_ptr<MapPoint> Ptr;
		unsigned long id_;
		static unsigned long factory_id_;
		bool good_;
		Vector3d pos_;
		Vector3d norm_;
		Mat descriptor_;

		list<Frame*> observed_frames_; // 可以看到该点的所有帧

		int matched_times_;
		int visible_times_;
	public:
		MapPoint();
		MapPoint(unsigned long id, const Vector3d& position,const Vector3d& norm, Frame* frame = nullptr, const Mat& descriptor=Mat());
		~MapPoint();

		inline cv::Point3f getPositionCV() const {
			return cv::Point3f(pos_(0), pos_(1), pos_(2));
		}
		static MapPoint::Ptr createMapPoint();
		static MapPoint::Ptr createMapPoint(
			const Vector3d& pos_world,
			const Vector3d& norm,
			const Mat& descriptor,
			Frame* frame
		);
	};

}


