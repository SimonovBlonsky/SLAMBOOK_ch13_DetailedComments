#ifndef MYSLAM_BACKEND_H
#define MYSLAM_BACKEND_H
#include"myslam/common_include.h"
#include"myslam/Map.h"
#include"myslam/Frame.h"

namespace myslam{
    class Map;
    class Backend
    {
    public:
        typedef std::shared_ptr<Backend> Ptr;

        // 构造函数中启动优化线程并挂起
        Backend();

        // 设置左右目的相机，用于获得内外参
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            cam_left_ = left;
            cam_right_ = right;
        }
        // 设置地图
        void SetMap(std::shared_ptr<Map> map)
        {
            map_ = map;
        }
        // 触发地图更新, 启动优化
        void UpdateMap();
        // 关闭后端线程
        void Stop();

    /* 私有函数 */
    private:
        // 后端线程
        void BackendLoop();

        // 优化给定的关键帧和路标点
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    /* 成员变量 */
    private:
        std::shared_ptr<Map> map_;

        // 后端线程
        std::thread backend_thread_;
        std::mutex data_mutex_;
        // 条件变量, 在多线程运行的时候提供等待条件
        std::condition_variable map_update_;

        // 原子操作, 保证只有一个线程访问这个变量
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };
}
#endif