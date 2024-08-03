#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/Camera.h"
#include "myslam/common_include.h"
#include "myslam/Frame.h"

/* 数据集读取 */
namespace myslam
{
    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Dataset> Ptr;
        
        // 构造函数, 从指定路径读取数据集
        Dataset(const std::string &dataset_path): dataset_path_(dataset_path) {}

        // 初始化, 返回是否成功
        bool Init();

        // 创建并返回下一帧的输入
        Frame::Ptr NextFrame();

        // 根据id获取相机信息
        Camera::Ptr GetCamera(const int camera_id) const
        {
            return cameras_.at(camera_id);      // 访问相机的vector
        }
    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };
}
#endif