//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name) {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor_Bilinear_interpolate(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        if (u_img > 0.f && u_img < width && v_img > 0.f && v_img < height) {
            Eigen::Vector2f lb((int)u_img, (int)v_img);
            Eigen::Vector2f lt(lb.x(), lb.y() + 1.f), rb(lb.x() + 1.f, lb.y()), rt(lb.x() + 1.f, lb.y() + 1.f);
            float s = u_img - lb.x(), t = v_img - lb.y();
            auto color0 = image_data.at<cv::Vec3b>(lb.y(), lb.x()) * (1.f - s) + image_data.at<cv::Vec3b>(rb.y(), rb.x()) * s;
            auto color1 = image_data.at<cv::Vec3b>(lt.y(), lt.x()) * (1.f - s) + image_data.at<cv::Vec3b>(rt.y(), rt.x()) * s;
            auto color = color0 * (1.f - t) + color1 * t;
            return Eigen::Vector3f(color[0], color[1], color[2]);
            // Eigen::Vector2f lt((int)u_img, (int)v_img), rt((int)(u_img + 1.f), (int)v_img), lb((int)u_img, (int)(v_img + 1.f)), rb((int)(u_img + 1.f), (int)(v_img + 1.f));
            // float k_rb = std::abs((u_img - lt.x()) * (v_img - lt.y()));
            // float k_lb = std::abs((u_img - rt.x()) * (v_img - rt.y()));
            // float k_rt = std::abs((u_img - lb.x()) * (v_img - lb.y()));
            // float k_lt = std::abs((u_img - rb.x()) * (v_img - rb.y()));
            // auto color_lt_cv = image_data.at<cv::Vec3b>(lt.x(), lt.y());
            // Eigen::Vector3f color_lt(color_lt_cv[0], color_lt_cv[1], color_lt_cv[2]);
            // auto color_rt_cv = image_data.at<cv::Vec3b>(rt.x(), rt.y());
            // Eigen::Vector3f color_rt(color_rt_cv[0], color_rt_cv[1], color_rt_cv[2]);
            // auto color_lb_cv = image_data.at<cv::Vec3b>(lb.x(), lb.y());
            // Eigen::Vector3f color_lb(color_lb_cv[0], color_lb_cv[1], color_lb_cv[2]);
            // auto color_rb_cv = image_data.at<cv::Vec3b>(rb.x(), rb.y());
            // Eigen::Vector3f color_rb(color_rb_cv[0], color_rb_cv[1], color_rb_cv[2]);
            // return k_lt * color_lt + k_rt * color_rt + k_lb * color_lb + k_rb * color_rb;
        } else {
            auto color = image_data.at<cv::Vec3b>(v_img, u_img);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
    }
};
#endif //RASTERIZER_TEXTURE_H
