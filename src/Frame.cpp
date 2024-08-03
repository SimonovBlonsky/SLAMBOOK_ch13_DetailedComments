#include"myslam/Frame.h"

namespace myslam
{
    // 构造函数, 初始化Frame的数据
    Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
    : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right){}

    //  工厂设计模式, 一共有两个工厂, 一个是keyframe的工厂, 一个是生产frame的工厂
    void Frame::SetKeyFrame()
    {
        // 分配关键帧id
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

    // 工厂构建模式, 分配id. 暂时不懂
    Frame::Ptr Frame::CreateFrame()
    {
        static long factory_id = 0;
        // 创建一个新的Frame
        Frame::Ptr new_frame(new Frame);
        new_frame->id_ = factory_id++;
        return new_frame;
    }
}