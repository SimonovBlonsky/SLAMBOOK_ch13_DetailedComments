#include"myslam/VisualOdometry.h"
#include <chrono>
#include "myslam/Config.h"

namespace myslam
{
    bool VisualOdometry::Init()     // 初始化VO
    {
        // 打开配置文件失败
        if(Config::SetParameterFile(config_file_path_) == false) return false;
    
        dataset_ = Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));

        CHECK_EQ(dataset_->Init(), true);     // 检测数据集是否加载成功

        // 初始化VO各个对象
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdometry::Run()     // Start VO in the dataset
    {
        while(1)
        {
            LOG(INFO) << "VO is running";
            if(Step() == false)
            {
                break;
            }
        }
        backend_->Stop();
        viewer_->Close();

        LOG(INFO) << "VO exit. ";
    }

    bool VisualOdometry::Step()        // Make a step forward in dataset
    {
        // 从数据集中拿到Frame对象
        Frame::Ptr new_frame = dataset_->NextFrame();

        // 下一帧加载不出来, VO退出
        if (new_frame == nullptr) return false;

        auto t1 = std::chrono::steady_clock::now();

        // 将新拿到的Frame放入VO前端
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        LOG(INFO) << "VO cost time: " << time_used.count() << "seconds";
        return success;
    }
}