#ifndef MYSLAM_GEOMETRIC_ALGORITHM_H
#define MYSLAM_GEOMETRIC_ALGORITHM_H
#include "myslam/common_include.h"
namespace myslam
{
    /**
     * linear triangulation with SVD, 线性三角化
     * 传入两个图片上关键点的相机归一化坐标, 得出三角化的点
     * @param poses     poses,
     * @param points    points in normalized plane
     * @param pt_world  triangulated point in the world
     * @return true if success
     */

    inline bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
    {   
        // 每个观测会得到两个方程, 因此是2 * poses.size()
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        /*
        Ax = 0
              [- T0 -]
        Pc = d[- T1 -]Pw
              [- T2 -]
        */
        for (size_t i = 0; i<poses.size(); ++i)
        {
            Mat34 m = poses[i].matrix3x4();     // 逐个读取pose
            /*
            位姿矩阵, 3*4
            ([[- T0 -],
              [- T1 -],
              [- T2 -]])
            
            从矩阵 `A` 的第 `2 * i` 行、第 0 列开始，取一个 1 行 4 列的子矩阵
            ([[ , , , ,],
              [ , , , ,],
              [ , , , ,],
              [ , , , ,]])
               ^
               | select
            */
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
            
            auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
            // 解取最后一维
            pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();  //head<3>表示取前3个元素

            if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
            {
                // 解质量不好, 放弃
                return true;
            }
            return false;
        }
    }

    /**
     * convert point coordinates from cv::Point2f to Vec2
     */
    inline Vec2 toVec2(const cv::Point2f p)
    {
        return Vec2(p.x, p.y);
    }
}
#endif