#include "common_include.h"
#include "MapPoint.h"
#include "Frame.h"
namespace myslam {

	class Map {
	public:
		typedef std::shared_ptr<Map> Ptr;
		unordered_map<unsigned long, MapPoint::Ptr> map_points_; // all landmarks;
		unordered_map<unsigned long, Frame::Ptr> keyframes_;// all key-frames;

		Map() {}

		void insertKeyFrame(Frame::Ptr frame);
		void insertMapPoint(MapPoint::Ptr map_point);
	};

}