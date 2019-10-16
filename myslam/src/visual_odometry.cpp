#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost\timer.hpp>

#include "Config.h"
#include "visual_odometry.h"
#include "g2o_types.h"
namespace myslam {
	VisualOdometry::VisualOdometry() :
		state_(INITIALIZING), ref_(nullptr), curr_(nullptr), map_(new Map), num_lost_(0), num_inliers_(0)
	{
		/*num_of_features_ = Config::get<int>("number_of_features");
		scale_factor_ = Config::get<double>("scale_factor");
		level_pyramid_ = Config::get<int>("level_pyramid");
		match_ratio_ = Config::get<float>("match_ratio");
		max_num_lost_ = Config::get<float>("max_num_lost");
		min_inliers_ = Config::get<int>("min_inliers");
		key_frame_min_rot = Config::get<double>("keyframe_rotation");
		key_frame_min_trans = Config::get<double>("keyframe_translation");*/

		num_of_features_ = Config::config_->number_of_features;
		scale_factor_ = Config::config_->scale_factor;
		level_pyramid_ = Config::config_->level_pyramid;
		match_ratio_ = Config::config_->match_ratio;
		max_num_lost_ = Config::config_->max_num_lost;
		min_inliers_ = Config::config_->min_inliers;
		key_frame_min_rot = Config::config_->key_frame_min_rot;
		key_frame_min_trans = Config::config_->key_frame_min_trans;

		orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
	}

	VisualOdometry::~VisualOdometry()
	{

	}
	void VisualOdometry::extractKeyPoints()
	{
		orb_->detect(curr_->color_, keypoints_curr_);
	}

	void VisualOdometry::computeDescriptors()
	{
		orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
	}

	void VisualOdometry::featureMatching() {
		vector<cv::DMatch> matches;
		cv::BFMatcher matcher(cv::NORM_HAMMING);
		matcher.match(descriptors_ref_, descriptors_curr_, matches);
		float min_dis = std::min_element(
			matches.begin(), matches.end(),
			[](const cv::DMatch& m1, const cv::DMatch& m2) {
			return m1.distance < m2.distance;
		})->distance;

		feature_matches_.clear();
		for (cv::DMatch& m : matches)
		{
			if (m.distance <  max<float>(min_dis*match_ratio_, 30))
			{
				feature_matches_.push_back(m);
			}
		}
		cout << "good matches: " << feature_matches_.size() << endl;

	}

	void VisualOdometry::poseEstimationPnP()
	{
		vector<cv::Point3f> pts3d;
		vector<cv::Point2f> pts2d;

		for (cv::DMatch m : feature_matches_) {
			pts3d.push_back(pts_3d_ref_[m.queryIdx]);
			pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
		}

		Mat K = (cv::Mat_<double>(3, 3) <<
			ref_->camera_->fx_, 0, ref_->camera_->cx_,
			0, ref_->camera_->fy_, ref_->camera_->cy_,
			0, 0, 1);
		Mat rvec, tvec, inliers,r_mat;
		cv::solvePnPRansac(pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

		cout << "pnp inliers: " << num_inliers_ << endl;

		num_inliers_ = inliers.rows;

	
		// 非线性优化
		if(true)
		{
			typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 2>> Block;
			Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
			Block* solver_ptr = new Block(linearSolver);
			g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
			g2o::SparseOptimizer optimizer;
			optimizer.setAlgorithm(solver);
			//添加节点
			g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
			pose->setId(0);
			pose->setEstimate(g2o::SE3Quat(
				T_c_r_estimated_.rotationMatrix(), T_c_r_estimated_.translation()
			));
			//pose->setEstimate(g2o::SE3Quat(R, T)); // 设置初始值
			optimizer.addVertex(pose);
			//添加边
			for (int i = 0; i < inliers.rows; i++)
			{
				int index = inliers.at<int>(i, 0);
				EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
				edge->setId(i);
				edge->setVertex(0, pose);
				edge->camera_ = curr_->camera_.get();
				edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z);
				edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
				edge->setInformation(Eigen::Matrix2d::Identity());
				optimizer.addEdge(edge);
			}
			optimizer.initializeOptimization();
			optimizer.optimize(10);


			T_c_r_estimated_ = SE3d(
				pose->estimate().rotation(),
				pose->estimate().translation()
			);
		}
		else {
			//转换为R矩阵
			cv::Rodrigues(rvec, r_mat);
			Eigen::Matrix3d R;
			for (size_t i = 0; i < 3; i++)
			{
				for (size_t j = 0; j < 3; j++)
				{
					R(i, j) = r_mat.at<double>(i, j);
				}
			}
			Vector3d T(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
			T_c_r_estimated_ = SE3d(
				SO3d(R),
				T
			);
		}
		
	}

	void VisualOdometry::setRef3DPoints()
	{
		pts_3d_ref_.clear();
		descriptors_ref_ = Mat();
		for (size_t i = 0; i < keypoints_curr_.size(); i++)
		{
			double d = ref_->findDepth(keypoints_curr_[i]);
			if (d>0)
			{
				Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
				pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
				descriptors_ref_.push_back(descriptors_curr_.row(i));

			}
		}
	}

	void VisualOdometry::addKeyFrame()
	{
		cout << "adding a key-frame" << endl;
		map_->insertKeyFrame(curr_);
	}

	bool VisualOdometry::checkEstimatedPose()
	{
		if (num_inliers_ < min_inliers_)
		{
			cout << "reject because inlier is too small: " << num_inliers_ << endl;
			return false;
		}
		Sophus::Vector6d d = T_c_r_estimated_.log();

		if (d.norm() > 5.0)
		{
			cout << "reject because motion is too large: " << d.norm() << endl;
			return false;
		}
		return true;
	}

	bool VisualOdometry::checkKeyFrame()
	{
		Sophus::Vector6d d = T_c_r_estimated_.log();
		Vector3d trans = d.head<3>();
		Vector3d rot = d.tail<3>();
		if (rot.norm() >key_frame_min_rot || trans.norm() >key_frame_min_trans)
			return true;
		return false;
	}

	bool VisualOdometry::addFrame(Frame::Ptr frame) {
		switch (state_)
		{
		case VisualOdometry::INITIALIZING:
		{
			state_ = OK;
			curr_ = ref_ = frame;
			map_->insertKeyFrame(frame);
			extractKeyPoints();
			computeDescriptors();
			setRef3DPoints();
			break;
		}
		case VisualOdometry::OK:
		{
			curr_ = frame;
			extractKeyPoints();
			computeDescriptors();
			featureMatching();
			//pnp
			poseEstimationPnP();
			if (checkEstimatedPose())
			{
				curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
				ref_ = curr_;
				setRef3DPoints();
				num_lost_ = 0;
				if (checkKeyFrame())
				{
					addKeyFrame();
				}
			}
			else {
				num_lost_++;
				if (num_lost_ > max_num_lost_)
				{
					state_ = LOST;
				}
				return false;
			}
			break;
		}
		case VisualOdometry::LOST:
		{
			cout << "vo has lost." << endl;
			break;
		}
		default:
			break;
		}
		return true;

	}
}