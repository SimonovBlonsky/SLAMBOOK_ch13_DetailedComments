#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H
#include <thread>
#include <pangolin/pangolin.h>

#include "myslam/common_include.h"
#include "myslam/Frame.h"
#include "myslam/Map.h"

namespace myslam
{
    /* 可视化 */
    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        // 传入地图
        void SetMap(Map::Ptr map)
        {
            map_ = map;
        }

        // 关闭可视化
        void Close();

        // 增加一个当前帧
        void AddCurrentFrame(Frame::Ptr current_frame);

        // 更新地图
        void UpdateMap();
    private:
        // 线程loop
        void ThreadLoop();

        // 显示图像帧
        void DrawFrame(Frame::Ptr frame, const float *color);

        // 显示地图点
        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

        // 在当前图像帧中显示特征点
        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        // 数据锁
        std::mutex viewer_date_mutex_;
    };
}
#endif