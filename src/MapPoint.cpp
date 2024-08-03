#include "myslam/MapPoint.h"
#include "myslam/Feature.h"

namespace myslam
{
    // 根据传入的单个特征点, 移除特征点对地图点的观测
    void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);

        // 使用迭代器iter遍历observation_, 找到传入feature对应的特征点地址
        for (auto iter = observations_.begin(); iter != observations_.end(); iter++)
        {
            // weak_ptr中的lock()会先判断其监视的shared_ptr有没有被销毁, 若销毁, 则返回空的shared_ptr, 若没有则返回shared_ptr
            if (iter->lock() == feature)
            {
                // 从observation中删除feature
                observations_.erase(iter);
                // 对于传入的特征点, 删除其对应的地图点
                feature->map_point_.reset();
                // 本地图点被观测到的次数-1
                observed_times_--;
                break;
            }
        }
    }

    // 使用工厂模式创建新地图点
    MapPoint::Ptr MapPoint::CreateNewMappoint()
    {
        static long factory_id = 0;
        MapPoint::Ptr new_mappoint(new MapPoint);
        new_mappoint->id_ = factory_id++; 
        /**
         * 等同于:
         * new_mappoint->id_ = factory_id;
         * factory_id++;
         *  */ 
        return new_mappoint;
    }
}