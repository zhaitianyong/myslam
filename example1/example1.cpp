// example1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "../myslam/src/Config.h"
#include "../myslam/src/visual_odometry.h"

int main(int argc, char** argv)
{
	/*if (argc != 2)
	{
		cout << "usage: run_vo parameter_file" << endl;
		return 1;
	}*/

	myslam::Config::setParameterFile("F:/MyGitHub/SLAM/Source/myslam/myslam/config/default.yaml");
	myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

	//string dataset_dir = myslam::Config::get<string>("dataset_dir");
	string dataset_dir = myslam::Config::config_->dataset_dir;
	cout << "dataset: " << dataset_dir << endl;
	ifstream fin(dataset_dir + "/associate.txt");
	if (!fin)
	{
		cout << "please generate the associate file called associate.txt!" << endl;
		return 1;
	}

	vector<string> rgb_files, depth_files;
	vector<double> rgb_times, depth_times;
	while (!fin.eof())
	{
		string rgb_time, rgb_file, depth_time, depth_file;
		fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
		cout << rgb_time;
		cout << " " << rgb_file;
		cout << " " << depth_time;
		cout << " " << depth_file << endl;

		rgb_times.push_back(atof(rgb_time.c_str()));
		depth_times.push_back(atof(depth_time.c_str()));
		rgb_files.push_back(dataset_dir + "/" + rgb_file);
		depth_files.push_back(dataset_dir + "/" + depth_file);

		if (fin.good() == false)
			break;
	}

	myslam::Camera::Ptr camera(new myslam::Camera(
		myslam::Config::config_->fx,
		myslam::Config::config_->fy,
		myslam::Config::config_->cx,
		myslam::Config::config_->cy,
		myslam::Config::config_->depth_scale
	));

	// visualization
	cv::viz::Viz3d vis("Visual Odometry");
	cv::viz::WCoordinateSystem world_coor(1.0), camera_coor(0.5);
	cv::Point3d cam_pos(0, -1.0, -1.0), cam_focal_point(0, 0, 0), cam_y_dir(0, 1, 0);
	cv::Affine3d cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
	vis.setViewerPose(cam_pose);

	world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 2.0);
	camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
	vis.showWidget("World", world_coor);
	vis.showWidget("Camera", camera_coor);

	cout << "read total " << rgb_files.size() << " entries" << endl;
	for (int i = 0; i<rgb_files.size(); i++)
	{
		Mat color = cv::imread(rgb_files[i]);
		Mat depth = cv::imread(depth_files[i], -1);
		if (color.data == nullptr || depth.data == nullptr)
			break;
		myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
		pFrame->camera_ = camera;
		pFrame->color_ = color;
		pFrame->depth_ = depth;
		pFrame->time_stamp_ = rgb_times[i];

		boost::timer timer;
		vo->addFrame(pFrame);
		cout << "VO costs time: " << timer.elapsed() << endl;

		if (vo->state_ == myslam::VisualOdometry::LOST)
			break;
		SE3d Tcw = pFrame->T_c_w_.inverse();

		// show the map and the camera pose 
		cv::Affine3d M(
			cv::Affine3d::Mat3(
				Tcw.rotationMatrix()(0, 0), Tcw.rotationMatrix()(0, 1), Tcw.rotationMatrix()(0, 2),
				Tcw.rotationMatrix()(1, 0), Tcw.rotationMatrix()(1, 1), Tcw.rotationMatrix()(1, 2),
				Tcw.rotationMatrix()(2, 0), Tcw.rotationMatrix()(2, 1), Tcw.rotationMatrix()(2, 2)
			),
			cv::Affine3d::Vec3(
				Tcw.translation()(0, 0), Tcw.translation()(1, 0), Tcw.translation()(2, 0)
			)
		);

		cv::imshow("image", color);
		cv::waitKey(1);
		vis.setWidgetPose("Camera", M);
		vis.spinOnce(1, false);
	}

	return 0;
}
