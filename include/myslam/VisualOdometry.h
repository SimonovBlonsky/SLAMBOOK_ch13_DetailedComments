#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/Backend.h"
#include "myslam/common_include.h"
#include "myslam/Dataset.h"
#include "myslam/Frontend.h"
#include "myslam/Viewer.h"

namespace myslam
{
    class VisualOdometry
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<VisualOdometry> Ptr;

        VisualOdometry(std::string &config_path): config_file_path_(config_path){}

        bool Init();     // initialization

        void Run();     // Start VO in the dataset

        bool Step();        // Make a step forward in dataset
    
        // 获取前端状态
        FrontendStatus GetFrontendStatus() const
        {
            return frontend_->GetStatus();
        }

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;
        Dataset::Ptr dataset_ = nullptr;
    };
}
#endif