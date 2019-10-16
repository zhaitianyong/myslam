#include "Config.h"
namespace myslam {


	shared_ptr<Config> Config::config_ = nullptr;


	Config::~Config()
	{
		if (file_.isOpened())
		{
			file_.release();
		}
	}

	void Config::setParameterFile(const std::string & filename)
	{
		if (config_ == nullptr) {
			config_ = shared_ptr<Config>(new Config());
		}
		cv::FileStorage fs(filename, cv::FileStorage::READ);
		//config_->file_ = fs;

		if (!fs.isOpened())
		{
			std::cerr << "parameter file " << filename << " does not exist." << std::endl;
			config_->file_.release();
			return;
		}
		config_->dataset_dir = fs["dataset_dir"];
		config_->fx = fs["camera.fx"];
		config_->fy = fs["camera.fy"];
		config_->cx = fs["camera.cx"];
		config_->cy = fs["camera.cy"];
		config_->depth_scale = fs["camera.depth_scale"];

		config_->number_of_features = fs["number_of_features"];
		config_->scale_factor = fs["scale_factor"];
		config_->level_pyramid = fs["level_pyramid"];
		config_->match_ratio = fs["match_ratio"];
		config_->max_num_lost = fs["max_num_lost"];
		config_->min_inliers = fs["min_inliers"];
		config_->key_frame_min_rot = fs["keyframe_rotation"];
		config_->key_frame_min_trans = fs["keyframe_translation"];
		config_->map_point_erase_ratio = fs["map_point_erase_ratio"];
	}

}
