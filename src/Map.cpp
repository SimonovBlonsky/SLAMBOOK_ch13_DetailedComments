#include "myslam/Map.h"
#include "myslam/Feature.h"

namespace myslam
{
     // 在地图中增加一个关键帧
    void Map::InsertKeyFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        // 先寻找frame对应的keyframe_id_有没有包含在地图的keyframes_里面
        // keyframes_中不存在当前frame对应的keyframe id
        if (keyframes_.find(frame->keyframe_id_) == keyframes_.end())
        {
            keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
            active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        }
        // 存在
        else
        {
            keyframes_[frame->keyframe_id_] = frame;
            active_keyframes_[frame->keyframe_id_] = frame;
        }

        // 添加完地图点之后, 检查一遍active_keyframe_有没有超过最大值
        if (active_keyframes_.size() > num_active_keyframes)
        {
            RemoveOldKeyframe();
        }
    }
    
    // 在地图中增加一个地图点
    void Map::InsertMapPoint(MapPoint::Ptr map_point)
    {
        if (landmarks_.find(map_point->id_) == landmarks_.end())
        {
            landmarks_.insert(std::make_pair(map_point->id_, map_point));
            active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
        }
        else
        {
            landmarks_[map_point->id_] = map_point;
            active_landmarks_[map_point->id_] = map_point;
        }
    }

    // 清理地图中观测数量为0的点
    void Map::CleanMap()
    {
        int cnt_landmark_removed = 0;
        for(auto iter = active_landmarks_.begin(); iter != landmarks_.end(); )
        {
            if(iter->second->observed_times_ == 0)
            {
                iter = active_landmarks_.erase(iter);      // .erase()会返回iter的下一个元素的迭代器, 因此iter不需要++
                cnt_landmark_removed++;
            }
            else
            {
                ++iter;
            }
        }
        LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
    }

    // 将旧的关键帧设置为不活跃状态
    void Map::RemoveOldKeyframe()
    {
        // 计算两帧之间位姿关系的距离, 来剔除关键帧, 策略有很多种
        // 此处的做法是, 如果存在很近的关键帧, 优先删除最近的, 否则删除最远的
        if (current_frame_ = nullptr) return;

        double max_dis = 0, min_dis = 9999;
        double max_kf_id = 0, min_kf_id = 0;

        auto Twc = current_frame_->pose().inverse();

        // 在active_keyframe当中, 找到最远和最近的keyframe id
        for(auto &kf : active_keyframes_)
        {
            // ape = ||log(Ti^-1 * Tj)||_2
            auto dis = (kf.second->pose() * Twc).log().norm();
            if(dis > max_dis)
            {
                max_dis = dis;
                max_kf_id = kf.first;
            }
            if(dis < min_dis)
            {
                min_dis = dis;
                min_kf_id = kf.first;
            }
        }

        const double min_dis_th = 0.2;      // 最小距离阈值
        Frame::Ptr frame_to_remove = nullptr;
        if (min_dis < min_dis_th)
        {
            frame_to_remove = keyframes_.at(min_kf_id);
        }
        else
        {
            frame_to_remove = keyframes_.at(max_kf_id);
        }

        LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;

        // 删除关键帧
        active_keyframes_.erase(frame_to_remove->keyframe_id_);

        // 根据需要删除的关键帧中的feature, 删除其对应的观测
        for(auto feature : frame_to_remove->features_left_)
        {
            auto mp = feature->map_point_.lock();
            if (mp)
            {
                mp->RemoveObservation(feature);
            }

        }

        CleanMap();

    }
}