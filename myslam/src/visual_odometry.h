#pragma once
#include "common_include.h"
#include "Map.h"
#include <opencv2\features2d.hpp>
#include <opencv2\calib3d.hpp>
namespace myslam {

	class VisualOdometry {
	public:
		typedef shared_ptr<VisualOdometry> Ptr;
		enum VOState {
			INITIALIZING = -1,
			OK = 0,
			LOST
		};

		VOState state_;
		Map::Ptr map_;
		Frame::Ptr ref_;
		Frame::Ptr curr_;

		cv::Ptr<cv::ORB> orb_; 
		vector<cv::Point3f> pts_3d_ref_;
		vector<cv::KeyPoint> keypoints_curr_;
		Mat descriptors_curr_;
		Mat descriptors_ref_; // descriptor in reference frame
		vector<cv::DMatch> feature_matches_;


		SE3d T_c_r_estimated_; // the estimated pose of current frame
		int num_inliers_; // number of inlier features in icp
		int num_lost_; // number of lost times

		// parameters
		int num_of_features_; // number of features
		double scale_factor_; // scale in image pyramid
		int level_pyramid_; // number of pyramid levels
		float match_ratio_; // ratio for selecting good matches
		int max_num_lost_; // max number of continuous lost times
		int min_inliers_; // minimum inliers
		
		double key_frame_min_rot; // minimal rotation of two key-frames
		double key_frame_min_trans; // minimal translation of two key-frames

		public: // functions
			VisualOdometry();
			~VisualOdometry();
		
			bool addFrame(Frame::Ptr frame);
		
		protected:
			// inner operation
			void extractKeyPoints();
			void computeDescriptors();
			void featureMatching();
			void poseEstimationPnP();
			void setRef3DPoints();
		
			void addKeyFrame();
			bool checkEstimatedPose();
			bool checkKeyFrame();

	};

}