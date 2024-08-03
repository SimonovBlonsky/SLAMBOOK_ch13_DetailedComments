#include "myslam/Dataset.h"
#include "myslam/Frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

namespace myslam
{
    // 初始化数据集, 读入相机内外参
    bool Dataset::Init()
    {
        // 读取标定文件
        std::ifstream fin(dataset_path_ + "/calib.txt");

        // 读取标定文件失败
        if (!fin)
        {
            LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt";
            return false;
        }
        /**
         * 标定文件格式:
         * P0: P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, P0_9, P0_10, P0_11, P0_12
           P1: P1_1, ...................................................................
           P2: P2_1, ...................................................................
           P3: P3_1,....................................................................
           Tr: velodyne, 激光雷达, 不用管
           此处的Pi(i = 0,1,2,3)并不是相机内参矩阵, 而是相机i的内参矩阵 * 相机0相对于相机i的外参矩阵
           因此 Pi = K(cami to pixel) * [R(cam0 to cami) t(cam0 to cami)]
           Pi = [K t]
         */

        // 一共4行
        for(int i = 0; i < 4; ++i)
        {
            char camera_name[3];     //读取前3个字符P0: 
            for (int k = 0; k<3;++k)
            {
                fin >> camera_name[k];
            }

            // 一个相机是12个参数
            double projection_data[12];
            for (int k = 0; k < 12; ++k)
            {
                fin >> projection_data[k];
            }

            Mat33 K;
            K << projection_data[0], projection_data[1], projection_data[2],
                projection_data[4], projection_data[5], projection_data[6],
                projection_data[8], projection_data[9], projection_data[10];
            Vec3 t;
            t << projection_data[3], projection_data[7], projection_data[11];

            // ? 不管了
            t = K.inverse() * t;
            K = K * 0.5;

            //数据集初始化时, 创建一个新的Camera对象
            // t.norm(): baseline
            Camera::Ptr new_camera(new Camera(
                K(0, 0), K(1, 1), K(0, 2), K(1, 2), t.norm(), SE3(SO3(), t)
            ));

            cameras_.push_back(new_camera);
        }
        fin.close();
        return true;
    }

    // 读取下一个图片, image.png -> Frame类, 交给VO前端处理
    Frame::Ptr Dataset::NextFrame()
    {   
        // 设置格式
        boost::format fmt("%s/image_%d/%06d.png");

        cv::Mat image_left, image_right;

        image_left = cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);
        image_right = cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(), cv::IMREAD_GRAYSCALE);

        if (image_left.data == nullptr || image_right.data == nullptr)
        {
            LOG(WARNING) << "cannot find images at index" << current_image_index_;
            return nullptr;     // 为何不用current_image_index_++?
        }

        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);    

        // 创建一个新Frame, 传入左、右目图像数据
        auto new_frame = Frame::CreateFrame();
        new_frame->left_img_ = image_left_resized;
        new_frame->right_img_ = image_right_resized;
        current_image_index_++;
        return new_frame;
    }
}