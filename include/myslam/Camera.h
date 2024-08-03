#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H
#include"common_include.h"
namespace myslam
{
    class Camera
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Camera> Ptr;

        // 相机参数
        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
        double baseline_ = 0;

        SE3 pose_;      // 位姿
        SE3 pose_inv_;      // 位姿的逆

        Camera(){}

        // 传入相机内外参数的构造函数
        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose):
        fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline)
        {
            pose_ = pose;
            pose_inv_ = pose.inverse();
        }

        // 获取相机位姿
        SE3 pose() const
        {
            return pose_;
        }

        // 获取相机内参
        Mat33 K() const
        {
            Mat33 k;
            k << fx_,  0,  cx_,
                 0,  fy_,  cy_,
                 0,    0,    1;
            return k;
        }

        /*各种坐标变换, p为点, T为变换矩阵*/

        // world -> camera 世界到相机
        Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);
        // camera -> world 相机到世界
        Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);

        // camera -> pixel 相机到像素
        Vec2 camera2pixel(const Vec3 &p_c);
        // pixel -> camera 像素到相机
        Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
       
        // pixel -> world 像素到世界
        Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
        // world -> pixel 世界到像素
        Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
    };
}
#endif