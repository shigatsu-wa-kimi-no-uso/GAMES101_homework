//
// Created by goksu on 4/6/19.
//

#pragma once
#ifndef RASTERIZER_H
#define RASTERIZER_H
#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "global.hpp"
#include "OBJ_Loader.h"
#include "Shader.hpp"
#include "Triangle.hpp"

namespace rst
{
    enum class Buffers
    {
        Color = 1,
        Depth = 2,
        Depth_SSAA = 4 
    };

    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    enum class Primitive
    {
        Line,
        Triangle
    };

    /*
     * For the curious : The draw function takes two buffer id's as its arguments. These two structs
     * make sure that if you mix up with their orders, the compiler won't compile it.
     * Aka : Type safety
     * */
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

    class rasterizer
    {
    public:
        rasterizer(int w, int h);
        pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);


  
        void set_object_loader(const objl::Loader& loader) { obj_loader = loader; }
        void set_viewport(const Eigen::Matrix4f& vp);
        void set_pixel(const Vector2i &point, const Eigen::Vector3f &color);
        void set_fragment_color(const Vector2i& point,const int frag_order, const Eigen::Vector3f& color);
        void clear(Buffers buff);

        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
        void draw(Shader shader);

        std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

    private:
        void draw_line(Eigen::Vector3f begin, Eigen::Vector3f end);
        void rasterize(int x, int y, Eigen::Vector4f(&screen_coords)[3], Shader& shader);
        void rasterize_with_ssaa(int x, int y, Eigen::Vector4f(&screen_coords)[3], Shader& shader);
        void rasterize_with_ssaa_optimized(int x, int y, Eigen::Vector4f(&screen_coords)[3], Shader& shader);

        void rasterize_triangle(Eigen::Vector4f (&screen_coords)[3], Shader& shader);
        void blend_color();
        // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER
      
    private:

        int normal_id = -1;
        objl::Loader obj_loader;
        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;
        std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

        Eigen::Matrix4f viewport;


        std::function<Eigen::Vector3f(fragment_shader_payload)> fragment_shader;
        std::function<Eigen::Vector3f(vertex_shader_payload)> vertex_shader;

        std::vector<Eigen::Vector3f> frame_buf;
        std::vector<float> depth_buf;
        std::vector<Eigen::Matrix<float,3,4>> frame_buf_ssaa;   //3x4矩阵,列和行分别为2x2 SSAA 4个片元的RGB颜色
        std::vector<Eigen::Vector4f> depth_buf_ssaa; //应用SSAA的Z-buffer,2x2时,每个像素对应4个子像素的深度
        int get_index(int x, int y);

        int width, height;

        int next_id = 0;
        int get_next_id() { return next_id++; }
    };
}

#endif