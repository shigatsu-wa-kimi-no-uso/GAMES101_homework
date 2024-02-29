//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(){}
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;
    /* 
    * v+----+----+
    * 2| 12 | 22 |
    *  +----+----+
    * 1| 11 | 21 |
    *  +----+----+
    *     1   2  u
    * interpolate: (u,v)
    * c11,c21 -> cu1
    * c12,c22 -> cu2
    * cu1,cu2 -> cuv
    */
    Eigen::Vector3f getColorBilinear(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        if (u_img < 0) u_img = 0;
        if (v_img < 0) v_img = 0;
        if (u_img >= width) u_img = width - 1;
        if (v_img >= height) v_img = height - 1;
        u = u_img;
        v = v_img;
        int u1 = u, v1 = v;
        int u2 = u1 + 1, v2 = v1 + 1;
        Eigen::Vector3f c11 = _getColor(u1, v1);
        Eigen::Vector3f c21 = _getColor(u2, v1);
        Eigen::Vector3f c12 = _getColor(u1, v2);
        Eigen::Vector3f c22 = _getColor(u2, v2);
        float ratio_u = (u - u1) / (u2 - u1); //±‡“Î∆˜ª·”≈ªØ
        Eigen::Vector3f cu1 = (c21 - c11) * ratio_u + c11;
        Eigen::Vector3f cu2 = (c22 - c12) * ratio_u + c12;
        float ratio_v = (v - v1) / (v2 - v1);
        Eigen::Vector3f cuv = (cu2 - cu1) * ratio_v + cu1;
        return cuv;
    }

    Eigen::Vector3f _getColor(int c, int r) {
        if (c < 0) c = 0;
        if (r < 0) r = 0;
        if (c >= width) c = width - 1;
        if (r >= height) r = height - 1;
        auto color = image_data.at<cv::Vec3b>(r,c);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        if (u_img < 0) u_img = 0;
        if (v_img < 0) v_img = 0;
        if (u_img >= width) u_img = width - 1;
        if (v_img >= height) v_img = height - 1;
        return _getColor(u_img, v_img);
    }

};
#endif //RASTERIZER_TEXTURE_H
