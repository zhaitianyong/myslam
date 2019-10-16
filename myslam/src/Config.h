#pragma once
#include "common_include.h"
namespace myslam {
	class Config
	{
	public:
		static std::shared_ptr<Config> config_;
	private:
		
		cv::FileStorage file_;
		Config() {};

	public:
		
		~Config();

		static void setParameterFile(const std::string& filename);


		template<typename T>
		static T get(const std::string& key) {
			std::cout << key << endl;
			if (config_->file_.isOpened())
			{
				config_->file_[key];
			}
			std::cout << key << endl;

			return T();
		}

	public:
		int number_of_features = 500;
		double scale_factor = 1.2;
		int level_pyramid = 8;
		float match_ratio = 2.0;
		float max_num_lost = 10.0;
		int min_inliers = 10;
		double key_frame_min_rot = 0.1;
		double key_frame_min_trans = 0.1;
		string dataset_dir;
		float fx, fy, cx, cy, depth_scale;
	};

}


