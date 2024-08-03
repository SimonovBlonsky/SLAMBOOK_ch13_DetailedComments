#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H
#include"common_include.h"
#include<memory>
#include<opencv2/features2d.hpp>


namespace myslam{

    struct Frame;
    struct MapPoint;
    /* 特征数据结构 */
    struct Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Feature> Ptr;

        // Frame持有了Feature的shared_ptr, 若Feature再持有Frame的shared_ptr, 则可能导致两者相互引用, 智能指针无法自动析构。
        std::weak_ptr<Frame> frame_;            // 持有该feature的frame

        cv::KeyPoint position_;          // 2D提取位置

        std::weak_ptr<MapPoint> map_point_;         // 关联地图点

        bool is_outlier_ = false;           // 是否为异常点, 默认为false

        bool is_on_left_image_ = true;          // 标识特征是否提取在左图, false为右图
    // Data members
    public:
        Feature(){}
        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp):frame_(frame),position_(kp) {}
    };
}
#endif