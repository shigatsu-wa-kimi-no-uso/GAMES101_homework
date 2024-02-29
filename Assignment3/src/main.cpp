#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
        0, 1, 0, 0,
        -sin(angle), 0, cos(angle), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
        0, 2.5, 0, 0,
        0, 0, 2.5, 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    // 远近与[-1,1]的关系: zFar --> -1 zNear --> 1 
    Eigen::Matrix4f projection;
    float f = -zFar, n = -zNear;
    float t = std::tan(MY_PI * (eye_fov / 2.0) / 180.0) * std::abs(n);
    float b = -t, r = t * aspect_ratio, l = -r;
    Eigen::Matrix4f ortho, trans, scale, persp2ortho;
    scale <<
        2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    trans <<
        1, 0, 0, -(r + l) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    persp2ortho <<
        n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -(n * f),
        0, 0, 1, 0;
    ortho = scale * trans;
    projection = ortho * persp2ortho;
    return projection;
}

Eigen::Matrix4f get_viewport_matrix(float width,float height, float zNear, float zFar)
{
    //注意: 如果对z轴操作,则可能改变z轴值的大小符号与远近的关系
    //此变换中, ndc_z = -1 --> screen_z=zNear, ndc_z = 1 --> screen_z=zFar, 0 < zNear < zFar
    float f1 = (zFar - zNear) / 2.0;
    float f2 = (zFar + zNear) / 2.0;
    Eigen::Matrix4f t,s,t2;
    t <<
        1, 0, 0, 1,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
    s <<
        0.5 * width, 0, 0, 0,
        0, 0.5 * height, 0, 0,
        0, 0, f1, 0,
        0, 0, 0, 1;
    t2 <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, f2,
        0, 0, 0, 1;
    return t2 * s * t;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    float view_vz[3] = {payload.view_coords[0].z(), payload.view_coords[1].z() ,payload.view_coords[2].z()};
    Eigen::Vector4f normal_interpolated = get_viewspace_bary_interpolated_value(payload.bar, payload.view_normals,view_vz);
    Eigen::Vector3f return_color = (normal_interpolated.head(3).normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result = return_color * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    // vec,axis均为单位向量, vec与axis夹角小于90度
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    // ambient(env) + diffuse(Lambert) + specular(Phong)
    // k in [0,1]
    float view_vz[3] = { payload.view_coords[0].z(), payload.view_coords[1].z() ,payload.view_coords[2].z() };
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    //Eigen::Vector3f kd = get_bary_interpolated_value(payload.bar, payload.colors) / 255.0;  //颜色归一化
    //Eigen::Vector3f reflect_point = get_bary_interpolated_value(payload.bar, payload.view_coords).head(3);
    //Eigen::Vector3f normal = get_bary_interpolated_value(payload.bar, payload.view_normals).head(3).normalized();
    Eigen::Vector3f kd = get_viewspace_bary_interpolated_value(payload.bar, payload.colors,view_vz)/255.0;
    Eigen::Vector3f reflect_point = get_viewspace_bary_interpolated_value(payload.bar,  payload.view_coords,view_vz ).head(3);
    Eigen::Vector3f normal = get_viewspace_bary_interpolated_value(payload.bar, payload.view_normals,view_vz).head(3).normalized();
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);


    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 }; //多光源
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 }; // Ia
    Eigen::Vector3f eye_pos{ 0, 0, 10 }; //给定眼睛位置,但在view space下, 眼睛位置应为摄像头位置,固定在原点

    float p = 150;

    const Eigen::Vector3f(&colors)[3] = payload.colors;
    const Eigen::Vector4f(&view_coords)[3] = payload.view_coords;
    const Eigen::Vector4f(&normals)[3] = payload.view_normals;

    Eigen::Vector3f result_color = { 0, 0, 0 };
    Eigen::Vector3f view_vec = (Eigen::Vector3f(0, 0, 0) - reflect_point).normalized();
    // I = Iamb + Idiff + Ispec = Ka*Ia + Kd*(I/r^2)*max(0,cos<normal,light>) + Ks*(I/r^2)*max(0,cos<normal,half>)^p

  
    Eigen::Vector3f i_amb = ka.cwiseProduct(amb_light_intensity);
    result_color += i_amb;
    for (auto& l : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        float r2 = (l.position - reflect_point).squaredNorm();
        Eigen::Vector3f intensity_arrived = l.intensity / r2;
        Eigen::Vector3f l_vec = (l.position - reflect_point).normalized();
        Eigen::Vector3f half_vec = (view_vec + l_vec).normalized();
        float cos_n_l = normal.dot(l_vec);
        float cos_n_h = normal.dot(half_vec);
        Eigen::Vector3f i_diff = kd.cwiseProduct(intensity_arrived) * std::max(0.f, cos_n_l);
        Eigen::Vector3f i_spec = ks.cwiseProduct(intensity_arrived) * std::pow(std::max(0.f, cos_n_h), p);
        result_color += i_diff + i_spec;
    }

    return result_color * 255.f;
}


Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = { 0, 0, 0 };
    float view_vz[3] = { payload.view_coords[0].z(), payload.view_coords[1].z() ,payload.view_coords[2].z() };
    Eigen::Vector3f texture_color;
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        Vector2f uv = get_viewspace_bary_interpolated_value(payload.bar, payload.tex_coords,view_vz);
        texture_color = payload.texture->getColor(uv.x(), uv.y());
    }

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 }; //给定眼睛位置,但在view space下, 眼睛位置应为摄像头位置,固定在原点

    float p = 150;

    Eigen::Vector3f reflect_point = get_viewspace_bary_interpolated_value(payload.bar, payload.view_coords,view_vz).head(3);
    Eigen::Vector3f normal = get_viewspace_bary_interpolated_value(payload.bar, payload.view_normals, view_vz).head(3).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };
    Eigen::Vector3f view_vec = (Eigen::Vector3f(0,0,0) - reflect_point).normalized();

    Eigen::Vector3f i_amb = ka.cwiseProduct(amb_light_intensity);
    result_color += i_amb;
    for (auto& l : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        float r2 = (l.position - reflect_point).squaredNorm();
        Eigen::Vector3f intensity_arrived = l.intensity / r2;
        Eigen::Vector3f l_vec = (l.position - reflect_point).normalized();
        Eigen::Vector3f half_vec = (view_vec + l_vec).normalized();
        float cos_n_l = normal.dot(l_vec);
        float cos_n_h = normal.dot(half_vec);
        Eigen::Vector3f i_diff = kd.cwiseProduct(intensity_arrived) * std::max(0.f, cos_n_l);
        Eigen::Vector3f i_spec = ks.cwiseProduct(intensity_arrived) * std::pow(std::max(0.f, cos_n_h), p);
        result_color += i_diff + i_spec;
    }

    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    float view_vz[3] = { payload.view_coords[0].z(), payload.view_coords[1].z() ,payload.view_coords[2].z() };
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = get_viewspace_bary_interpolated_value(payload.bar, payload.colors, view_vz) / 255.0;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);
    Vector2f uv;
    float width=0,height=0;

    auto h = [payload](float u, float v) {
        return payload.texture->getColor(u, v).norm();
    };

    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        uv = get_viewspace_bary_interpolated_value(payload.bar, payload.tex_coords, view_vz);
        width = payload.texture->width, height = payload.texture->height;
    }
    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    const Eigen::Vector3f(&colors)[3] = payload.colors;
    const Eigen::Vector4f(&view_coords)[3] = payload.view_coords;
    const Eigen::Vector2f(&tex_coords)[3] = payload.tex_coords;
    const Eigen::Vector4f(&normals)[3] = payload.view_normals;

    float kh = 0.2, kn = 0.1;
   // float kh = 1, kn = 1;
    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    //games论坛求法,特殊求法,非准确TBN
    Eigen::Vector3f result_color = { 0, 0, 0 };
    /*
    const Eigen::Vector3f& n = get_viewspace_bary_interpolated_value(payload.bar, payload.view_normals, view_vz).head(3).normalized();
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    Eigen::Vector3f t1(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    Eigen::Vector3f b1 = n.cross(t1);
    Eigen::Matrix3f TBN; //TBN矩阵: 将纹理坐标对应到模型空间中
    TBN <<
        t1.x(), b1.x(), n.x(),
        t1.y(), b1.y(), n.y(),
        t1.z(), b1.z(), n.z();
    float u = uv.x(), v = uv.y();
    float du = kh * kn * (h(u + 1.0 / width, v) - h(u, v));
    float dv = kh * kn * (h(u, v + 1.0 / height) - h(u, v));
    Eigen::Vector3f n_ts(-du, -dv, 1);
    Eigen::Vector3f n_ws = (TBN * n_ts).normalized();
    result_color += n_ws;*/
    
    // [t,b]=[e1,e2][delta_u1,delta_u2]^(-1)
    for (int i = 0; i < 3;i++) {
        const Eigen::Vector3f& n = normals[i].head(3).normalized();
        const Eigen::Vector3f& e1 = (view_coords[(i+1)%3] - view_coords[i]).head(3);
        const Eigen::Vector3f& e2 = (view_coords[(i+2)%3] - view_coords[i]).head(3);
        const Eigen::Vector2f& delta_uv1 = tex_coords[(i + 1) % 3] - tex_coords[i];
        const Eigen::Vector2f& delta_uv2 = tex_coords[(i + 2) % 3] - tex_coords[i];
        const float ratio = 1.0 / (delta_uv1.x() * delta_uv2.y() - delta_uv2.x() * delta_uv1.y());
        Eigen::Matrix<float, 3, 2> e;
        e<<e1, e2;
        //std::cout << e1 << "\n";
        //std::cout << e2 << "\n";
        //std::cout << e << "\n";
        //传统求法
        Eigen::Vector3f t = ratio * e * Eigen::Vector2f(delta_uv2.y(), -delta_uv1.y());
       // Eigen::Vector3f b = ratio * e * Eigen::Vector2f(-delta_uv2.x(),delta_uv1.x());
        Eigen::Vector3f tp = (t - t.dot(n) * n).normalized();
       // Eigen::Vector3f bp = (b - b.dot(n) * n - b.dot(tp) * tp).normalized();
        Eigen::Vector3f bp = n.cross(tp).normalized();
       // std::cout << bp << "\n" << bp2 << "\n"<< tp.cross(n).normalized();
        //tb = e * coords.inverse();
        //Eigen::Vector3f t = tb.col(0),b=tb.col(2);
        //Eigen::Vector3f bp = (b - b.dot(n) * n - b.dot(tp) * tp).normalized();
        Eigen::Matrix3f tbn;
        tbn << tp, bp, n;
        //std::cout << tbn << "\n" << TBN;
        // dU = kh * kn * (h(u+1/w,v)-h(u,v))
        // dV = kh * kn * (h(u,v+1/h)-h(u,v))
        // Vector ln = (-dU, -dV, 1)
        // Normal n = normalize(TBN * ln)
        float u = uv.x(), v = uv.y();
        float du = kh * kn * (h(u + 1.0 / width, v) - h(u, v));
        float dv = kh * kn * (h(u, v + 1.0 / height) - h(u, v));
        Eigen::Vector3f n_ts(-du, -dv, 1);    
        Eigen::Vector3f n_ws = (tbn * n_ts).normalized();
        result_color += n_ws/3.0;
    }

    return result_color * 255.f;
}

Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    float view_vz[3] = { payload.view_coords[0].z(), payload.view_coords[1].z() ,payload.view_coords[2].z() };
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = get_viewspace_bary_interpolated_value(payload.bar, payload.colors, view_vz) / 255.0;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    //Eigen::Vector3f kd = get_bary_interpolated_value(payload.bar, payload.colors) / 255.0;  //颜色归一化
    //Eigen::Vector3f reflect_point = get_bary_interpolated_value(payload.bar, payload.view_coords).head(3);
    //Eigen::Vector3f normal = get_bary_interpolated_value(payload.bar, payload.view_normals).head(3).normalized();
    Eigen::Vector3f reflect_point = get_viewspace_bary_interpolated_value(payload.bar, payload.view_coords, view_vz).head(3);
    Vector2f uv;
    float width = 0, height = 0;

    auto h = [payload](float u, float v) {
        return payload.texture->getColor(u, v).norm();
    };

    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        uv = get_viewspace_bary_interpolated_value(payload.bar, payload.tex_coords, view_vz);
        width = payload.texture->width, height = payload.texture->height;
    }
    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    const Eigen::Vector3f(&colors)[3] = payload.colors;
    const Eigen::Vector4f(&view_coords)[3] = payload.view_coords;
    const Eigen::Vector2f(&tex_coords)[3] = payload.tex_coords;
    const Eigen::Vector4f(&normals)[3] = payload.view_normals;
    Eigen::Vector3f normal(0, 0, 0);
    float kh = 0.2, kn = 0.1;
    // float kh = 1, kn = 1;
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    //games论坛求法,特殊求法,非准确TBN
    
    const Eigen::Vector3f& n = get_viewspace_bary_interpolated_value(payload.bar, payload.view_normals, view_vz).head(3).normalized();
    /*
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    Eigen::Vector3f t1(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    Eigen::Vector3f b1 = n.cross(t1);
    Eigen::Matrix3f TBN; //TBN矩阵: 将纹理坐标对应到模型空间中
    TBN <<
        t1.x(), b1.x(), n.x(),
        t1.y(), b1.y(), n.y(),
        t1.z(), b1.z(), n.z();
    float u = uv.x(), v = uv.y();
    float du = kh * kn * (h(u + 1.0 / width, v) - h(u, v));
    float dv = kh * kn * (h(u, v + 1.0 / height) - h(u, v));
    Eigen::Vector3f n_ts(-du, -dv, 1);
    Eigen::Vector3f n_ws = (TBN * n_ts).normalized();
    normal = n_ws;
    */
    // [t,b]=[e1,e2][delta_u1,delta_u2]^(-1)
    
    for (int i = 0; i < 3; i++) {
        const Eigen::Vector3f& n = normals[i].head(3).normalized();
        const Eigen::Vector3f& e1 = (view_coords[(i + 1) % 3] - view_coords[i]).head(3);
        const Eigen::Vector3f& e2 = (view_coords[(i + 2) % 3] - view_coords[i]).head(3);
        const Eigen::Vector2f& delta_uv1 = tex_coords[(i + 1) % 3] - tex_coords[i];
        const Eigen::Vector2f& delta_uv2 = tex_coords[(i + 2) % 3] - tex_coords[i];
        const float ratio = 1.0 / (delta_uv1.x() * delta_uv2.y() - delta_uv2.x() * delta_uv1.y());
        Eigen::Matrix<float, 3, 2> e;
        e << e1, e2;
        //std::cout << e1 << "\n";
        //std::cout << e2 << "\n";
        //std::cout << e << "\n";
        //传统求法
        Eigen::Vector3f t = ratio * e * Eigen::Vector2f(delta_uv2.y(), -delta_uv1.y());
        // Eigen::Vector3f b = ratio * e * Eigen::Vector2f(-delta_uv2.x(),delta_uv1.x());
        Eigen::Vector3f tp = (t - t.dot(n) * n).normalized();
        // Eigen::Vector3f bp = (b - b.dot(n) * n - b.dot(tp) * tp).normalized();
        Eigen::Vector3f bp = n.cross(tp).normalized();
        // std::cout << bp << "\n" << bp2 << "\n"<< tp.cross(n).normalized();
         //tb = e * coords.inverse();
         //Eigen::Vector3f t = tb.col(0),b=tb.col(2);
         //Eigen::Vector3f bp = (b - b.dot(n) * n - b.dot(tp) * tp).normalized();
        Eigen::Matrix3f tbn;
        tbn << tp, bp, n;
        //std::cout << tbn << "\n" << TBN;
        // dU = kh * kn * (h(u+1/w,v)-h(u,v))
        // dV = kh * kn * (h(u,v+1/h)-h(u,v))
        // Vector ln = (-dU, -dV, 1)
        // Normal n = normalize(TBN * ln)
        float u = uv.x(), v = uv.y();
        float du = kh * kn * (h(u + 1.0 / width, v) - h(u, v));
        float dv = kh * kn * (h(u, v + 1.0 / height) - h(u, v));
        Eigen::Vector3f n_ts(-du, -dv, 1);
        Eigen::Vector3f n_ws = (tbn * n_ts).normalized();
        normal += n_ws;
    }
    normal.normalize();
    //Position p = p + kn * n * h(u, v)
    reflect_point += (kn * n * h(uv.x(), uv.y())); //在法线n的方向上增长 kn*h(u, v) 高度
    Eigen::Vector3f result_color = { 0, 0, 0 };
    Eigen::Vector3f view_vec = (Eigen::Vector3f(0, 0, 0) - reflect_point).normalized();
    Eigen::Vector3f i_amb = ka.cwiseProduct(amb_light_intensity);
    result_color += i_amb;
    // I = Iamb + Idiff + Ispec = Ka*Ia + Kd*(I/r^2)*max(0,cos<normal,light>) + Ks*(I/r^2)*max(0,cos<normal,half>)^p
    for (auto& l : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        float r2 = (l.position - reflect_point).squaredNorm();
        Eigen::Vector3f intensity_arrived = l.intensity / r2;
        Eigen::Vector3f l_vec = (l.position - reflect_point).normalized();
        Eigen::Vector3f half_vec = (view_vec + l_vec).normalized();
        float cos_n_l = normal.dot(l_vec);
        float cos_n_h = normal.dot(half_vec);
        Eigen::Vector3f i_diff = kd.cwiseProduct(intensity_arrived) * std::max(0.f, cos_n_l);
        Eigen::Vector3f i_spec = ks.cwiseProduct(intensity_arrived) * std::pow(std::max(0.f, cos_n_h), p);
        result_color += i_diff + i_spec;
    }

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;
    std::string filename = "output.png";
    std::string obj_path = "C:/Users/Administrator/Documents/GAMES101_homework_proj/GAMES101_homework/Assignment3/src/models/spot/";
    objl::Loader loader;
    rst::rasterizer r(700, 700);

    // Load .obj File
    bool loadout = loader.LoadFile("C:/Users/Administrator/Documents/GAMES101_homework_proj/GAMES101_homework/Assignment3/src/models/spot/spot_triangulated_good.obj");

    for(auto mesh:loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    auto texture_path = "spot_texture.png";
    Texture texture = Texture(obj_path + texture_path);

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            //active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            //r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            //active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            //active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            //active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};


    int key = 0;
    int frame_count = 0;

    r.set_object_loader(loader);

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        Shader shader;
        shader.set_model(get_model_matrix(angle));
        shader.set_view(get_view_matrix(eye_pos));
        shader.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
        shader.set_texture(texture);
        shader.set_fragment_shader(active_shader);
        r.set_viewport(get_viewport_matrix(700, 700, 0.1, 50));
        r.draw(shader);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(filename, image);
        return 0;
    }
    
    while(key != 27)
    {
        Shader shader;
        r.clear(rst::Buffers::Color | rst::Buffers::Depth | rst::Buffers::Depth_SSAA);
        shader.set_model(get_model_matrix(angle));
        shader.set_view(get_view_matrix(eye_pos));
        shader.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));
        shader.set_texture(texture);
        shader.set_fragment_shader(active_shader);
        r.set_viewport(get_viewport_matrix(700, 700, 0.1, 50));
        r.draw(shader);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
