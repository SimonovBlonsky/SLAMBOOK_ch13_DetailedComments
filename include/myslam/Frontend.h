#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include"myslam/common_include.h"
#include"myslam/Frame.h"
#include"myslam/Map.h"
#include"myslam/Camera.h"
namespace myslam
{
    class Backend;
    class Viewer;

    enum class FrontendStatus
    {
        INITING, TRACKING_GOOD, TRACKING_BAD, LOST
    };

    /* 前端类 */
    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        /*外部接口*/

        // 添加图像帧
        bool AddFrame(Frame::Ptr frame);

        // SetMap
        void SetMap(Map::Ptr map)
        {
            map_ = map;
        };

        // 链接backend。SetBackend. 由于没有包含backend的头文件, 所以在本文件中先做了backend的声明。
        void SetBackend(std::shared_ptr<Backend> backend)
        {
            backend_ = backend;
        }

        // SetViewer, 可视化
        void SetViewer(std::shared_ptr<Viewer> viewer)
        {
            viewer_ = viewer;
        }
        
        // 获取前端的状态
        FrontendStatus GetStatus() const
        {
            return status_;
        }

        // 设置相机
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        /*类私有成员函数, 控制前端操作*/
        
        /**
         * Track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         * Reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * Track with last frame, 对上一帧进行追踪
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * estimate current frame's pose, 估计当前帧的位姿
         * @return num of inliers
         */
        int EstimateCurrentPose();

        /**
         * set current frame as a keyframe and insert it into backend
         * 设置当前帧为关键帧, 并将其交给后端处理
         * @return true if success
         */
        bool InsertKeyFrame();

        /**
         * Try init the frontend with stereo images saved in current_frame_
         * 使用双目图像进行前端初始化, 并存储在current_frame_当中
         * @return true if success
         */
        bool StereoInit();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * 在左目图像中进行特征检测, 关键点存储于current_frame_当中
         * @return
         */
        int DetectFeatures();

        /**
         * Find the corresponding features in right image of current_frame_
         * 在右图中匹配左图检测到的关键点
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * Build the initial map with single image
         * 使用初始图像帧, 建立初始地图
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * Triangulate the 2D points in current frame
         * 当当前图像帧需要建立地图点时, 进行三角化
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * Set the features in keyframe as new observation of the map points
         * 将关键帧中检测的特征, 作为map points的新观测
         */
        void SetObservationsForKeyFrame();
    
    /* 数据 */
    private:
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr;    // 当前帧
        Frame::Ptr last_frame_ = nullptr;       // 上一帧
        Camera::Ptr camera_left_ = nullptr;     // 左目相机
        Camera::Ptr camera_right_ = nullptr;    // 右目相机

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        SE3 relative_motion_;       // 当前帧与上一帧的相对运动, 用于估计当前帧pose初值

        int tracking_inliers_ = 0;      // 用于验证新的关键帧

        // 参数
        int num_features_ = 200;        // 特征点数
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;        // 追踪状态需要的特征点数
        int num_features_tracking_bad_ = 20;        // Bad追踪状态对应的特征点数
        int num_features_needed_for_keyframe_ = 80;     // 关键帧需要的特征点数

        // 特征点检测模块
        cv::Ptr<cv::GFTTDetector> gftt_;
    };
}
#endif
