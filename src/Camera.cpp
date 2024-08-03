#include "myslam/Camera.h"

namespace myslam
{
    /*各种坐标变换, p为点, T为变换矩阵*/

    // world -> camera 世界到相机
    Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &T_c_w)
    {
        return pose_ * T_c_w * p_w;     // Tcw是什么?
    }
    // camera -> world 相机到世界
    Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &T_c_w)
    {
        return T_c_w.inverse() * pose_inv_ * p_c;
    }

    // camera -> pixel 相机到像素
    Vec2 Camera::camera2pixel(const Vec3 &p_c)
    {
        return Vec2(
            fx_*p_c(0, 0) / p_c(2, 0) + cx_, 
            fy_*p_c(1, 0) / p_c(2, 0) + cy_
        );
    }
    // pixel -> camera 像素到相机
    Vec3 Camera::pixel2camera(const Vec2&p_p, double depth)    //注意此处不能depth=1, 否则会出现默认参数重定义的情况
    {
        return Vec3(
            (p_p(0, 0) - cx_) / fx_ * depth,
            (p_p(1, 0) - cy_) / fy_ * depth,
            depth
        );
    }
       
    // pixel -> world 像素到世界
    Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth)
    {
        return T_c_w.inverse() * pose_inv_ * Vec3(
            (p_p(0, 0) - cx_) / fx_,
            (p_p(1, 0) - cy_) / fy_,
            depth
        );
    }
    // world -> pixel 世界到像素
    Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &T_c_w)
    {
        Vec3 p_c = pose_ * T_c_w * p_w;
        return Vec2(
            fx_*p_c(0, 0) / p_c(2, 0) + cx_, 
            fy_*p_c(1, 0) / p_c(2, 0) + cy_
        );
    }
}