#include <opencv2/opencv.hpp>

#include "myslam/GeometricAlgorithm.h"
#include "myslam/Backend.h"
#include "myslam/Config.h"
#include "myslam/Feature.h"
#include "myslam/Frontend.h"
#include "myslam/GraphUtils.h"
#include "myslam/Map.h"
#include "myslam/Viewer.h"

namespace myslam
{
    // 前端的构造函数
    Frontend::Frontend()
    {
        // 特征点
        gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);

        num_features_init_ = Config::Get<int>("num_features_init");
        num_features_ = Config::Get<int>("num_features");
    }

    // 根据前端的不同状态, 采取不同的操作
    bool Frontend::AddFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;
        
        switch (status_)
        {
        case FrontendStatus::INITING:
            StereoInit();
            break;
        case FrontendStatus::TRACKING_GOOD:

        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            break;
        }
        last_frame_ = current_frame_;
        return true;
    }

    /**
    * Track in normal mode
    * 当检测到的特征点较少时, 对于TRACKING_BAD状态, 进行一次关键帧的插入
     * @return true if success
     */
    bool Frontend::Track()
    {
        if(last_frame_)
        {
            // 初始化时relative_motion为1
            current_frame_->SetPose(relative_motion_ * last_frame_->pose());
        }

        int num_track_last = TrackLastFrame();
        tracking_inliers_ = EstimateCurrentPose();

        if(tracking_inliers_ > num_features_tracking_)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
        } else if (tracking_inliers_ > num_features_tracking_bad_)
        {
            status_ = FrontendStatus::TRACKING_BAD;
        } else
        {
            status_ = FrontendStatus::LOST;
        }
        
        InsertKeyFrame();

        // 取本次的相对运动, 作为下一次位姿估计的初始值
        relative_motion_ = current_frame_->pose() * last_frame_->pose().inverse();

        if(viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
        }
        return true;
    }

    /**
     * Reset when lost
     * 本例中, reset还没有实现
     * @return true if success
     */
    bool Frontend::Reset()
    {
        LOG(INFO) << "Reset is not implemeted. ";
        return true;
    }

    /**
     * Track with last frame, 使用LK光流法对上一帧进行追踪
     * @return num of tracked points
     */
    int Frontend::TrackLastFrame()
    {
        // 整体结构类似左右目匹配, 但是此处匹配的是前一帧和当前帧的左目
        std::vector<cv::Point2f> kps_last, kps_current;
        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                auto mp = kp->map_point_.lock();
                auto px = camera_left_->world2pixel(mp->pos_, current_frame_->pose());
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.push_back(kp->position_.pt);
                kps_current.push_back(kp->position_.pt);
            }

            std::vector<uchar> status;
            Mat error;
            cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, kps_last, kps_current, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);

            int num_good_pts = 0;

            for(size_t i = 0; i<status.size(); ++i)
            {
                if(status[i])
                {
                    auto kp = cv::KeyPoint(kps_current[i], 7);
                    auto feature = Feature::Ptr(new Feature(current_frame_, kp));
                    // feature->is_on_left_image_ = true;
                    feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                    current_frame_->features_left_.push_back(feature);
                    num_good_pts++;
                }
            }

            LOG(INFO) << "Find " << num_good_pts << "in the last image. ";
            return num_good_pts;
        }
    }

    /**
     * estimate current frame's pose, PnP估计当前帧的位姿
     * @return num of inliers
     */
    int Frontend::EstimateCurrentPose()
    {
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;

        auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 初始化顶点。
        // Track当中, current_frame_->SetPose(relative_motion_ * last_frame_->pose());
        VertexPose *vertex_pose = new VertexPose();
        vertex_pose->setId(0);
        vertex_pose->setEstimate(current_frame_->pose_);
        optimizer.addVertex(vertex_pose);

        Mat33 K =camera_left_->K();

        // 初始化边
        int index =1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;
        
        // 添加边
        for(size_t i = 0; i<current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp)
            {
                features.push_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pos_, K);

                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.push_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }

        // 估计位姿
        const double chi2_th = 5.991;
        int cnt_outlier = 0;
        int iterations = 4;
        for(int j = 0; j<iterations;++j)
        {
            vertex_pose->setEstimate(current_frame_->pose());
            optimizer.initializeOptimization();
            optimizer.optimize(10);
            cnt_outlier = 0;

            for (size_t i = 0; i< edges.size(); ++i)
            {
                auto e = edges[i];

                if(features[i]->is_outlier_)
                {
                    e->computeError();
                }

                if(e->chi2()>chi2_th)
                {
                    features[i]->is_outlier_=true;
                }
                else
                {
                    features[i]->is_outlier_=false;
                }
                
                // 细节, 剔除了两轮外点之后就不需要核函数了
                if (iterations == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }
    
        LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/" << features.size() - cnt_outlier;

        for(auto &f : features)
        {
            if (f->is_outlier_)
            {
                f->map_point_.reset();
                f->is_outlier_ = false;
            }
        }

        return features.size() - cnt_outlier;
    }

        /**
         * set current frame as a keyframe and insert it into backend
         * 设置当前帧为关键帧, 并将其交给后端处理
         * @return true if success
         */
    bool Frontend::InsertKeyFrame()
    {
        // 追踪到的点小于一定值时, 才会插入关键帧
        if(tracking_inliers_ > num_features_needed_for_keyframe_)
        {
            return false;
        }

        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);

        LOG(INFO) << "Set frame" << current_frame_->id_ << "as keyframe " << current_frame_->keyframe_id_;

        SetObservationsForKeyFrame();

        DetectFeatures();
        FindFeaturesInRight();

        TriangulateNewPoints();

        backend_->UpdateMap();

        if(viewer_) viewer_->UpdateMap();

        return true;
    }

    /**
     * Try init the frontend with stereo images saved in current_frame_
     * 使用双目图像进行前端初始化, 并存储在current_frame_当中
     * @return true if success
     */
    bool Frontend::StereoInit()
    {
        int num_features_left = DetectFeatures();
        int num_coor_features = FindFeaturesInRight();

        // 左右匹配数不对, 初始化失败
        if(num_coor_features < num_features_left) return false;

        // 建立初始地图
        bool build_map_success = BuildInitMap();

        // 初始地图建立成功, 初始化成功
        if(build_map_success)
        {
            status_ = FrontendStatus::TRACKING_GOOD;
            if(viewer_)
            {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            return true;
        }
        return false;
    }

    /**
     * Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     * 在左目图像中进行特征检测, 关键点存储于current_frame_当中
     * @return
     */
    int Frontend::DetectFeatures()
    {
        // 创建一个单通道mask. 作用是若当前帧已存在特征点, 则在原本没有特征点的区域内检测特征点.
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);

        // 若当前帧已存在特征点, 则用mask 遮挡掉
        for (auto &feat :current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
            feat->position_.pt - cv::Point2f(10, 10), 0, cv::FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        gftt_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;

        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_, kp)));
            cnt_detected ++;
        }

        LOG(INFO) << "Detect " << cnt_detected << "new features";
        return cnt_detected;
    }

    /**
     * Find the corresponding features in right image of current_frame_
     * 使用LK光流在右图中匹配左图检测到的关键点
     * @return num of features found
     */
    int Frontend::FindFeaturesInRight()
    {
        std::vector<cv::Point2f> kps_left, kps_right;

        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->position_.pt);

            // 如果存在特征点 -> 地图点的观测, 则使用地图点的投影做初始估计
            auto mp = kp->map_point_.lock();
            if(mp)
            {
                auto px = camera_right_->world2pixel(mp->pos_, current_frame_->pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_right.push_back(kp->position_.pt);
            }
        }

        // 开始光流估计
        std::vector<uchar> status;
        Mat error;
        cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_, kps_left, kps_right, status, error, cv::Size(11, 11), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        // 统计匹配成功的特征点数
        int num_good_pts = 0;
        for (size_t i = 0; i<status.size(); ++i)
        {
            // 左右目匹配成功
            if(status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat(new Feature(current_frame_,kp));
                feat->is_on_left_image_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
            LOG(INFO) << "Find " << num_good_pts << "in the right image.";
            return num_good_pts;
        }
    }

    /**
     * Build the initial map with single image
     * 使用初始图像帧, 建立初始地图
     * @return true if succeed
     */
    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;

        for(size_t i=0;i<current_frame_->features_left_.size();++i)
        {
            if (current_frame_->features_right_[i] == nullptr) continue;

            std::vector<Vec3> points
            {
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                    current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                    current_frame_->features_right_[i]->position_.pt.y))
            };

            Vec3 pworld = Vec3::Zero();

            // 三角化操作
            bool triangulate_success = triangulation(poses, points, pworld);
            if(triangulate_success && pworld[2] > 0)
            {
                auto new_map_point = MapPoint::CreateNewMappoint();
                new_map_point->SetPos(pworld);

                // map -> feature -> frame
                new_map_point->AddObservation(current_frame_->features_left_[i]);
                new_map_point->AddObservation(current_frame_->features_right_[i]);
                
                // frame -> feature -> map
                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
            
                cnt_init_landmarks ++;
                map_->InsertMapPoint(new_map_point);
            }
        }

        // 初始化成功, 当前帧视为关键帧
        current_frame_->SetKeyFrame();
        map_->InsertKeyFrame(current_frame_);
        backend_->UpdateMap();
        LOG(INFO) << "Initial map created with " << cnt_init_landmarks << " map points";

        return true;
    }

    /**
     * Triangulate the 2D points in current frame
     * 当检测到关键帧, 需要建立新的地图点时, 进行三角化
     * @return num of triangulated points
     */
    int Frontend::TriangulateNewPoints()
    {
        // 获取相机位姿
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        // camera_left->pose()和Twc不是一个东西, 前者是固定的, 后者是运动物体的位姿
        SE3 current_pose_Twc = current_frame_->pose().inverse();
        
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {   
            // 在访问weak_ptr所指向的对象之前使用.expired()来检查对象是否还存在
            if (current_frame_->features_left_[i]->map_point_.expired() && current_frame_->features_right_[i] != nullptr)
            {
                std::vector<Vec3> points
                {
                    camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x, current_frame_->features_left_[i]->position_.pt.y)),
                    camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x, current_frame_->features_right_[i]->position_.pt.y))
                };
                Vec3 pworld = Vec3::Zero();

                bool success = triangulation(poses, points, pworld);
                if (success && pworld[2]>0)
                {
                    auto new_map_point = MapPoint::CreateNewMappoint();
                    pworld = current_pose_Twc * pworld;
                    new_map_point->SetPos(pworld);
                    new_map_point->AddObservation(current_frame_->features_left_[i]);

                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
            }
        }
        LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    /**
     * Set the features in keyframe as new observation of the map points
     * 为新关键帧中的关键点, 加入map points的观测
     */
    void Frontend::SetObservationsForKeyFrame()
    {
        for(auto &feat: current_frame_->features_left_)
        {
            auto mp = feat->map_point_.lock();
            if(mp)
            {
                mp->AddObservation(feat);
            }
        }
    }

}

