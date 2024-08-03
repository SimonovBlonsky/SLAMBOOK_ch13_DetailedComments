#include "myslam/Viewer.h"
#include "myslam/Feature.h"
#include "myslam/Frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam
{
    Viewer::Viewer()
    {
        viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
    }

        // 关闭可视化
    void Viewer::Close()
    {
        viewer_running_ = false;
        viewer_thread_.join();
    }

    // 增加一个当前帧
    void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
    {
        std::unique_lock<std::mutex> lck(viewer_date_mutex_);
        current_frame_ = current_frame;
    }

    // 更新地图
    void Viewer::UpdateMap()
    {
        std::unique_lock<std::mutex> lck(viewer_date_mutex_);

        // assert地图是否为空指针, 若条件不成立, 程序终止
        assert(map_ != nullptr);
        active_keyframes_ = map_->GetActiveKeyframes();
        active_landmarks_ = map_->GetActiveMapPoints();
        map_updated_ = true;
    }

    // 线程loop
    void Viewer::ThreadLoop()
    {
        pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        // 设置3D Handler
        pangolin::View& vis_display = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f);

        // 颜色数组
        const float blue[3] = {0, 0, 1};
        const float green[3] = {0, 1, 0};

        while(!pangolin::ShouldQuit() && viewer_running_)
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            vis_display.Activate(vis_camera);

            // 显示图片
            std::unique_lock<std::mutex> lock(viewer_date_mutex_);
            if(current_frame_)
            {
                DrawFrame(current_frame_, green);
                FollowCurrentFrame(vis_camera);

                cv::Mat img = PlotFrameImage();
                cv::imshow("image", img);
                cv::waitKey(1);
            }

            if(map_)
            {
                DrawMapPoints();
            }

            pangolin::FinishFrame();
            usleep(5000);
        }

        LOG(INFO) << "Stop Viewer. ";
    }

    // 显示图像帧
    void Viewer::DrawFrame(Frame::Ptr frame, const float *color)
    {
        SE3 Twc = frame->pose().inverse();
        // 图像显示参数
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        // 使用了模板方法cast来将Twc.matrix()返回的矩阵转换为float类型
        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat*)m.data());

        if(color == nullptr)
        {
            glColor3f(1, 0, 0);
        } else {
            glColor3f(color[0], color[1], color[2]);
        }

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    // 显示地图点
    void Viewer::DrawMapPoints()
    {
        const float red[3] = {1.0, 0, 0};
        for (auto &kf:active_keyframes_)
        {
            DrawFrame(kf.second, red);
        }

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto&landmark:active_landmarks_)
        {
            auto pos = landmark.second->Pos();
            glColor3f(red[0], red[1], red[2]);
            glVertex3d(pos[0], pos[1], pos[2]);
        }
        glEnd();
    }

    // 使相机跟随当前帧的位姿
    void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        SE3 Twc = current_frame_->pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    // 在当前图像帧中显示特征点
    cv::Mat Viewer::PlotFrameImage()
    {
        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
        for(size_t i =0 ; i < current_frame_->features_left_.size(); ++i)
        {
            // 当前帧中的特征存在对应的地图点
            if(current_frame_->features_left_[i]->map_point_.lock())
            {
                auto feat = current_frame_->features_left_[i];

                // 在特征点处画圆圈
                cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 255, 0), 2);
            }
        }
        return img_out;
    }
}