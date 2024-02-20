//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H
#include <eigen3/Eigen/Eigen>
#include "OBJ_Loader.h"
typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)


inline Eigen::Vector4f to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}


inline Eigen::Vector4f to_vec4(const objl::Vector3& v3, float w = 1.0f)
{
    return Eigen::Vector4f(v3.X,v3.Y,v3.Z,w);
}

inline Eigen::Vector3f to_eigen_vec3(const objl::Vector3& v3)
{
    return Eigen::Vector3f(v3.X, v3.Y, v3.Z);
}


inline Eigen::Vector2f to_eigen_vec2(const objl::Vector2& v2)
{
    return Eigen::Vector2f(v2.X, v2.Y);
}


template<class T>
inline T get_bary_interpolated_value(const Eigen::Vector3f& bar, const T(&v)[3]) {
    return bar.x() * v[0] + bar.y() * v[1] + bar.z() * v[2];
}


template<class T>
inline T get_viewspace_bary_interpolated_value(const Eigen::Vector3f& bar, const T(&v)[3], const float(&view_vz)[3]) {
    float view_z = 1.0 / (bar.x() / view_vz[0] + bar.y() / view_vz[1] + bar.z() / view_vz[2]);
    T view_attrib = bar.x()* v[0] / view_vz[0] + bar.y() * v[1] / view_vz[1] + bar.z() * v[2] / view_vz[2];
    return view_attrib / view_z;
}



#endif //RASTERIZER_GLOBAL_H
