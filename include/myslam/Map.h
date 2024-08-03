#pragma once
#ifndef MAP_H
#define MAP_H

#include"myslam/common_include.h"
#include"myslam/MapPoint.h"
#include"myslam/Frame.h"

namespace myslam
{
    // 需要让一个map类来持有Frame和MapPoint的对象
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;

        // 创建unordered_map容器, 根据地图点或关键帧的id查找地图点或关键帧
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map(){}
        
        void InsertKeyFrame(Frame::Ptr frame);      // 增加一个关键帧, 传入一个std::shared_ptr<Frame>智能指针
        
        void InsertMapPoint(MapPoint::Ptr map_point);       // 增加一个地图点

        // 获得所有地图点
        LandmarksType GetAllMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }

        // 获得所有关键帧
        KeyframesType GetAllKeyframes()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        // 获得激活地图点
        LandmarksType GetActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        // 获得激活关键帧
        KeyframesType GetActiveKeyframes()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        // 清理map中观测数量为0的点
        void CleanMap();

    private:
        // 将旧的关键帧设置为不活跃状态
        void RemoveOldKeyframe();
        
        std::mutex data_mutex_;      // 数据锁
        LandmarksType landmarks_;       // 所有地图点
        LandmarksType active_landmarks_;        // 所有激活的地图点. "激活"即滑窗
        KeyframesType keyframes_;       // 所有关键帧
        KeyframesType active_keyframes_;        // 所有激活的关键帧

        Frame::Ptr current_frame_ = nullptr;        // 当前帧设置为空

        // settings
        int num_active_keyframes = 7;       // 激活的关键帧数量. 此处设置滑窗的大小为7
    };
}

#endif