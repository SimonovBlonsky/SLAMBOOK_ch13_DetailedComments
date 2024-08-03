#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include"common_include.h"
#include"Feature.h"
#include"Camera.h"
namespace myslam{
    struct Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frame> Ptr;

        unsigned long id_ = 0;      //本帧id
        unsigned long keyframe_id_ = 0;     //关键帧id
        bool is_keyframe_ = false;      //是否为关键帧
        double time_stamp_ = 0;      // 时间戳

        SE3 pose_;      // Tcw形式pose, world -> camera, X' = Tcw * Xw. 默认为原点.
        std::mutex pose_mutex;      // Pose锁
        
        cv::Mat left_img_, right_img_;       // stereo images
        std::vector<std::shared_ptr<Feature>> features_left_;       // 左图中提取得到的特征
        std::vector<std::shared_ptr<Feature>> features_right_;      // 左图的特征关联到右图中的特征, 如果没有就设置为nullptr
    public:
        // Data members
        Frame(){}
        Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

        // 读取目前的帧的pose并加锁, 确保只有一个线程可以访问pose
        SE3 pose()
        {
            std::unique_lock<std::mutex> lck(pose_mutex);
            return pose_;
        }

        // 对当前帧的pose进行修改, 同样需要加锁
        void SetPose(const SE3 &pose)
        {
            std::unique_lock<std::mutex> lck(pose_mutex);
            pose_ = pose;
        }

        //设置关键帧并分配关键帧id
        void SetKeyFrame();

        // 工厂构建模式, 分配id. 暂时不懂
        static std::shared_ptr<Frame> CreateFrame();
    };
}
#endif