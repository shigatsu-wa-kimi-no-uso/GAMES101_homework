// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>
#include <math.h>
#include "rasterizer.hpp"

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector2f pnt(x, y), pntVec;
    Vector2f vertexes[3];
    vertexes[0] << _v[0].head(2);
    vertexes[1] << _v[1].head(2);
    vertexes[2] << _v[2].head(2);
    int flag = 0;
    for (int i = 0; i < 3; i++) {
        Vector2f sideVec = vertexes[(i+1)%3] - vertexes[i]; 
        pntVec = pnt - vertexes[i];
        if ( pntVec.x()*sideVec.y() - pntVec.y()*sideVec.x() > 0) { //叉乘结果有正负
            flag++;
        }
    }
    if (flag == 3 || flag == 0) {
        return true;    //三次结果同向时, flag只能为3或0
    } else {
        return false;
    }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        blend_color();
    }
}


static float get_z_interpolated(float alpha,float beta,float gamma,const std::array<Eigen::Vector4f,3>& v) {
    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    return z_interpolated;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    std::array<float, 3> coordx,coordy;
    std::transform(v.begin(), v.end(), coordx.begin(), [](Eigen::Vector4f& vec) { return vec.x(); });
    std::transform(v.begin(), v.end(), coordy.begin(), [](Eigen::Vector4f& vec) { return vec.y(); });
    int lft = *std::min_element(coordx.begin(),coordx.end()); //float -> int
    int rgt = *std::max_element(coordx.begin(), coordx.end()) + 1; //float -> int
    int top = *std::max_element(coordy.begin(), coordy.end()) + 1; //float -> int
    int btn = *std::min_element(coordy.begin(), coordy.end()); //float -> int
    for (int x = lft; x <= rgt; x++) {
        for (int y = btn; y <= top; y++) {
            //rasterize(x, y, t);
            rasterize_with_ssaa_optimized(x, y, t);
        }
    }

}



void rst::rasterizer::rasterize(int x, int y, const Triangle& t) {
    // using 4x4 super-sampling anti-aliasing
    // 像素中点为(x,y)
    auto v = t.toVector4();
    int index = get_index(x, y);
    if (insideTriangle(x /* + 0.5 */, y /* + 0.5 */, t.v)) {
        // If so, use the following code to get the interpolated z value.
        auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
        float z_interpolated = get_z_interpolated(alpha, beta, gamma, v);
        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
        // z值经过反转, depth_buf初始值为无穷大,越大越远
        float depth = -z_interpolated;
        if (depth_buf[index] > depth) {
            depth_buf[index] = depth;
            set_pixel(Vector3f(x, y, z_interpolated), t.getColor());    //假设三角形为单色,3个顶点颜色不同只按其中一个为准
        }
    }
}

void rst::rasterizer::rasterize_with_ssaa(int x,int y, const Triangle& t) {
    // using 2x2 super-sampling anti-aliasing
    // 像素中点为(x,y),则一个像素中4个子像素位置分别为(x-0.25,y-0.25),....
    Vector2f subpixels[4] = { Vector2f(x - 0.25,y - 0.25),Vector2f(x + 0.25,y - 0.25),Vector2f(x - 0.25,y + 0.25),Vector2f(x + 0.25,y + 0.25) };
    Vector3f finalColor(0, 0, 0);
    auto v = t.toVector4();
    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float z_interpolated = get_z_interpolated(alpha, beta, gamma, v);
    int index = get_index(x, y);
    for (int i = 0; i < 4; i++) {
        float sub_x = subpixels[i].x(), sub_y = subpixels[i].y();
        if (insideTriangle(sub_x /* + 0.5 */, sub_y /* + 0.5 */, t.v)) {
            // If so, use the following code to get the interpolated z value.
            auto [subalpha, subbeta, subgamma] = computeBarycentric2D(sub_x, sub_y, t.v);
            float sub_z_interpolated = get_z_interpolated(subalpha, subbeta, subgamma, v);
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            // z值经过反转, depth_buf初始值为无穷大,越大越远
            float depth = -sub_z_interpolated;
            if (depth_buf_ssaa[index][i] > depth) {
                depth_buf_ssaa[index][i] = depth;
                finalColor += t.getColor();
                continue;
            }
        }
        finalColor += frame_buf[index]; //否则采用背景色
    }
    finalColor /= 4.0; //4个子像素求平均
    set_pixel(Vector3f(x, y, z_interpolated), finalColor);    //假设三角形为单色,3个顶点颜色不同只按其中一个为准
}



void rst::rasterizer::rasterize_with_ssaa_optimized(int x, int y, const Triangle& t) {
    // using 2x2 super-sampling anti-aliasing
    // 像素中点为(x,y),则一个像素中4个子像素位置分别为(x-0.25,y-0.25),....
    //procedure: bounding box -> rasterize -> depth testing -> fragment shader -> write buffer
    //深度测试也可以在fragment shader过程之后
    Eigen::Vector2f subpixels[4] = { Eigen::Vector2f(x - 0.25,y - 0.25),Eigen::Vector2f(x + 0.25,y - 0.25),Eigen::Vector2f(x - 0.25,y + 0.25),Eigen::Vector2f(x + 0.25,y + 0.25) };
    Eigen::Vector3f frag_color(0, 0, 0);
    auto v = t.toVector4();
    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    int index = get_index(x, y);
    for (int i = 0; i < 4; i++) {
        float sub_x = subpixels[i].x(), sub_y = subpixels[i].y();
        if (insideTriangle(sub_x /* + 0.5 */, sub_y /* + 0.5 */, t.v)) {
            // If so, use the following code to get the interpolated z value.
            auto [subalpha, subbeta, subgamma] = computeBarycentric2D(sub_x, sub_y, t.v);
            Eigen::Vector3f bar(subalpha, subbeta, subgamma);
            //深度测试
            //z值以screen space为准, 不需要以view space为准
            float sub_z_interpolated = get_z_interpolated(subalpha, subbeta, subgamma, v);
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            // 注意:z值与远近的关系需由projection和viewport变换共同决定,viewport变换后,z值越小越远,经过反转为depth值,越大越远, depth_buf初始值为无穷大
            float depth = -sub_z_interpolated;
            if (depth_buf_ssaa[index][i] > depth) {
                depth_buf_ssaa[index][i] = depth;
                frag_color = t.getColor();
                set_fragment_color(Eigen::Vector2i(x, y), i, frag_color); //仅当此种情况才会设置颜色,其余情况保持buffer不变
            }
        }
    }
}

void rst::rasterizer::set_fragment_color(const Eigen::Vector2i& point, const int frag_order, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    //int ind = (height - point.y()) * width + point.x();
    int ind = get_index(point.x(), point.y());
    frame_buf_ssaa[ind].col(frag_order) = color;
}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
    if ((buff & rst::Buffers::Depth_SSAA) == rst::Buffers::Depth_SSAA)
    {
        Eigen::Vector4f vec;
        vec.fill(std::numeric_limits<float>::infinity());
        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), vec);
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Matrix<float, 3, 4>::Zero());
    }
}

void rst::rasterizer::blend_color() {
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            int index = get_index(x, y);
            Eigen::Matrix<float, 3, 4>& frag_colors = frame_buf_ssaa[index];
            frame_buf[index] = (frag_colors.col(0) + frag_colors.col(1) + frag_colors.col(2) + frag_colors.col(3)) / 4.0;
        }
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_ssaa.resize(w * h);
    frame_buf_ssaa.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = get_index(point.x(), point.y());
    frame_buf[ind] = color;

}

// clang-format on