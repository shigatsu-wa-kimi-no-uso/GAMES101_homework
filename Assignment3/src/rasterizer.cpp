//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include "Shader.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


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

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector2i point = Eigen::Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector2i point = Eigen::Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}



static bool insideTriangle(int x, int y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}




void rst::rasterizer::set_viewport(const Eigen::Matrix4f& vp)
{
    viewport = vp;
}

void rst::rasterizer::draw(Shader shader) {

    for (auto mesh : obj_loader.LoadedMeshes)
    {
        for (int i = 0; i < mesh.Vertices.size(); i += 3)
        {
            Eigen::Vector4f screen_coords[3];
            for (int j = 0; j < 3; j++)
            {
                //for each vertex: Vertex Shader -> Homogeneous divide -> Viewport transform
                //space transform: object space --MVP--> clipping space --divide w--> NDC --viewport trans--> screen space
                Eigen::Vector3f object_coord = to_eigen_vec3(mesh.Vertices[i + j].Position);
                Eigen::Vector2f tex_coord = to_eigen_vec2(mesh.Vertices[i + j].TextureCoordinate);
                Eigen::Vector3f normal = to_eigen_vec3(mesh.Vertices[i + j].Normal);
                Eigen::Vector3f color = Vector3f(148, 121.0, 92.0);
                Eigen::Vector4f clip_coord = shader.vertex_shader({ j,object_coord,tex_coord,normal,color });
                //ignore clipping operation
                Eigen::Vector4f ndc_coord = clip_coord/clip_coord.w();
                screen_coords[j] = viewport * ndc_coord;
            }
            rasterize_triangle(screen_coords,shader);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(Eigen::Vector4f (&screen_coords)[3], Shader& shader)
{
    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);
    std::array<float, 3> coordx, coordy;
    std::array<Eigen::Vector4f, 3> v = { screen_coords[0],screen_coords[1],screen_coords[2] };
    std::transform(v.begin(), v.end(), coordx.begin(), [](Eigen::Vector4f& vec) { return vec.x(); });
    std::transform(v.begin(), v.end(), coordy.begin(), [](Eigen::Vector4f& vec) { return vec.y(); });
    int lft = *std::min_element(coordx.begin(), coordx.end()); //float -> int
    int rgt = *std::max_element(coordx.begin(), coordx.end()) + 1; //float -> int
    int top = *std::max_element(coordy.begin(), coordy.end()) + 1; //float -> int
    int btn = *std::min_element(coordy.begin(), coordy.end()); //float -> int
    for (int x = lft; x <= rgt; x++) {
        for (int y = btn; y <= top; y++) {
            //rasterize(x, y, t);
            rasterize_with_ssaa(x, y, screen_coords,shader);
        }
    }
}


static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}



void rst::rasterizer::rasterize_with_ssaa(int x, int y, Eigen::Vector4f(&screen_coords)[3], Shader& shader) {
    // using 4x4 super-sampling anti-aliasing
    // 像素中点为(x,y),则一个像素中4个子像素位置分别为(x-0.25,y-0.25),....

    //procedure: bounding box -> rasterize -> depth testing -> fragment shader -> write buffer
    //深度测试也可以在fragment shader过程之后
    Vector2f subpixels[4] = { Vector2f(x - 0.25,y - 0.25),Vector2f(x + 0.25,y - 0.25),Vector2f(x - 0.25,y + 0.25),Vector2f(x + 0.25,y + 0.25) };
    Vector3f finalColor(0, 0, 0);
    auto [alpha, beta, gamma] = computeBarycentric2D(x, y, screen_coords);
    int index = get_index(x, y);
    for (int i = 0; i < 4; i++) {
        float sub_x = subpixels[i].x(), sub_y = subpixels[i].y();
        if (insideTriangle(sub_x /* + 0.5 */, sub_y /* + 0.5 */, screen_coords)) {
            // If so, use the following code to get the interpolated z value.
            auto [subalpha, subbeta, subgamma] = computeBarycentric2D(sub_x, sub_y, screen_coords);
            Vector3f bar(subalpha, subbeta,subgamma);
            //深度测试
            //z值以screen space为准, 不需要以view space为准
            float sub_z_interpolated = get_bary_interpolated_value(bar, { screen_coords[0].z(), screen_coords[1].z(), screen_coords[2].z() });
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
            // z值经过反转为depth值, depth_buf初始值为无穷大,越大越远
            float depth = -sub_z_interpolated;
            if (depth_buf_ssaa[index][i] > depth) {
                depth_buf_ssaa[index][i] = depth;
                finalColor += shader.fragment_shader(bar);
                continue;
            }
        }
        finalColor += frame_buf[index]; //否则采用背景色
    }
    finalColor /= 4.0; //4个子像素求平均
    set_pixel(Vector2i(x, y), finalColor);    //假设三角形为单色,3个顶点颜色不同只按其中一个为准
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
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    depth_buf_ssaa.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}
