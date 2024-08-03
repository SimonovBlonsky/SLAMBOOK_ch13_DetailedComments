#include "myslam/Backend.h"
#include "myslam/GeometricAlgorithm.h"
#include "myslam/Feature.h"
#include "myslam/GraphUtils.h"
#include "myslam/Map.h"
#include "myslam/MapPoint.h"
namespace myslam
{
    // 构造函数中启动优化线程并挂起
    Backend::Backend()
    {
        backend_running_.store(true);

        // 启动后端优化线程
        // 为什么bind要用this? 本人的理解是: bind绑定对象函数时, 需要传入一个指向对象本身的指针作为第一个参数。
        backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
    }

    // 后端线程
    void Backend::BackendLoop()
    {
        // 使用atomic.load()读取原子数据
        while(backend_running_.load())
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            map_update_.wait(lock);

            Map::KeyframesType active_kfs = map_->GetActiveKeyframes();
            Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();

            Optimize(active_kfs, active_landmarks);
        }
    }

    // 触发地图更新, 启动优化
    void Backend::UpdateMap()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        // 发出信号, 需要更新了
        map_update_.notify_one();
    }
    // 关闭后端线程
    void Backend::Stop()
    {
        backend_running_.store(false);
        // 关闭之前再进行一次update
        map_update_.notify_one();
        backend_thread_.join();
    }



    // 优化给定的关键帧和路标点
    void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
    {
        typedef g2o::BlockSolver_6_3 BlockSolvertype;
        typedef g2o::LinearSolverCSparse<BlockSolvertype::PoseMatrixType> LinearSolverType;
        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolvertype>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 将keyframe放入顶点表中
        std::map<unsigned long, VertexPose *> vertices;

        // 最大的关键帧id
        unsigned long max_kf_id = 0;
        for(auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;
            VertexPose *vertex_pose = new VertexPose();

            // 要注意定点对应的id是关键帧id
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->pose());
            if(kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }
            vertices.insert({kf->keyframe_id_, vertex_pose});
        }

        // K和左右外参. 此处的外参是相对于相机本身pose的投影矩阵, 仅含有一个t
        Mat33 K = cam_left_->K();
        SE3 left_ext = cam_left_->pose();
        SE3 right_ext = cam_right_->pose();
        // 边
        int index = 1;
        double chi2_th = 5.991;     // robust kernel 阈值

        // 路标顶点
        std::map<unsigned long, VertexXYZ*> vertices_landmarks;
        // 将特征点和边放在一起索引。这样做的目的是方便后面离群点的剔除
        std::map<EdgeProjection*, Feature::Ptr> edges_and_features;
    
        for(auto &landmark : landmarks)
        {
            // 过滤离群点
            if(landmark.second->is_outlier_) continue;

            unsigned long landmark_id = landmark.second->id_;

            // 获取observation, 即与该路标点有关的feature
            auto observations = landmark.second->GetObs();

            // 遍历observation
            for(auto &obs : observations)
            {
                if(obs.lock()==nullptr) continue;       // 对于weak_ptr一定要拿lock()检验

                auto feat = obs.lock();

                // 检测该特征是否为离群点
                if(feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
                // observation对应的frame
                auto frame = feat->frame_.lock();
                EdgeProjection *edge = nullptr;

                // 根据左右相机确定Edge传入的相机外参
                if(feat->is_on_left_image_)
                {
                    edge = new EdgeProjection(K, left_ext);
                } else {
                    edge = new EdgeProjection(K, right_ext);
                }

                // 如果landmark还没被加入优化, 则加入landmarks顶点
                if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ;
                    // 对于路标来说, observation就是坐标
                    v->setEstimate(landmark.second->Pos());
                    /**
                     * Pose1 max_kf_id landmark1
                     * |             | |
                     * [][][][][]...[]()()()()...()
                     */
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                // 设置边
                if(vertices.find(frame->keyframe_id_)!=vertices.end() && vertices_landmarks.find(landmark_id)!=vertices_landmarks.end())
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));
                    edge->setVertex(1, vertices_landmarks.at(landmark_id));
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(Mat22::Identity());        // 信息矩阵默认为单位阵
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index ++;
                } else {
                    delete edge;
                }
            }
        }

        // 进行优化
        optimizer.initializeOptimization();
        optimizer.optimize(10);
        
        // 根据内/外点比例, 确定门限值
        int cnt_outlier = 0, cnt_inlier = 0;
        int iteration = 0;
        int min_inlier_ratio = 0.5;
        while(iteration < 5)
        {
            cnt_outlier = 0;
            cnt_inlier = 0;

            for(auto &ef:edges_and_features)
            {
                if(ef.first->chi2()>chi2_th)
                {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
            }
            double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if(inlier_ratio > min_inlier_ratio)
            {
                break;
            } else {
                // 内点比例不够, 说明门限太低, 需要调高
                chi2_th*=2;
                iteration++;
            }
        }

        for(auto &ef : edges_and_features)
        {
            if(ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                ef.second->map_point_.lock()->RemoveObservation(ef.second);
            } else {
                ef.second->is_outlier_ = false;
            }  
        }

        LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << " / " << cnt_inlier;

        // 根据优化结果设定pose和landmark的位置
        for (auto &v: vertices)
        {
            keyframes.at(v.first)->SetPose(v.second->estimate());
        }
        for (auto &v: vertices_landmarks)
        {
            landmarks.at(v.first)->SetPos(v.second->estimate());
        }
    }
}
