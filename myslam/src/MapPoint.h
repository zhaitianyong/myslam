#pragma once
#include "common_include.h"
namespace myslam {
	class MapPoint
	{

	public:
		typedef shared_ptr<MapPoint> Ptr;
		unsigned long id_;
		Vector3d pos_;
		Vector3d norm_;
		Mat descriptor_;
		int observed_times_;
		int correct_times_;
	public:
		MapPoint();
		MapPoint(long id, Vector3d position, Vector3d norm);
		~MapPoint();

		static MapPoint::Ptr createMapPoint();
	};

}


