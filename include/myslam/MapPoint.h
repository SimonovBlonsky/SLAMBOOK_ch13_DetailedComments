#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H


#include"common_include.h"

namespace myslam
{
    struct Frame;
    struct Feature;
    struct MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<MapPoint> Ptr;
        
        unsigned long id_ = 0;      // 地图点id
        
        bool is_outlier_ = false;        // 是否为离群点
        
        Vec3 pos_ = Vec3::Zero();       // 地图点在世界坐标系下的坐标, 初始化为0
        
        std::mutex data_mutex_;     // 坐标的数据锁;
        
        int observed_times_ = 0;        // 被特征匹配算法观察到的次数

        // 地图点对应的特征, 一个地图点可能被许多特征点观察到, 所以使用std::list存储, 方便频繁插入及删除.
        std::list<std::weak_ptr<Feature>> observations_;       
    public:
        MapPoint(){}
        MapPoint(long id, Vec3 position): id_(id), pos_(position){}

        // 访问地图点的坐标
        Vec3 Pos()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return pos_;
        }

        // 设置地图点的坐标
        void SetPos(const Vec3 &pos)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            pos_ = pos;
        }
        
        // 若该地图点被新的特征点观测到, 则需要在该地图点内添加新的observations.
        void AddObservation(std::shared_ptr<Feature> feature)
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            observations_.push_back(feature);
            observed_times_ ++;
        }
        void RemoveObservation(std::shared_ptr<Feature> feature);

        // 获取观测特征点
        std::list<std::weak_ptr<Feature>> GetObs()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return observations_;
        }
        
        // Factory function
        static MapPoint::Ptr CreateNewMappoint();
    };
}

#endif
