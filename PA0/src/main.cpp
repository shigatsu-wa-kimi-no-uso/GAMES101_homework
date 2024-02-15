#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

using namespace Eigen;


Vector3f processVector(Vector3f p) {
    float pi4ths = 3.1415926 / 4.0;

    Matrix3f rotMat;
    rotMat << 
        cos(pi4ths), sin(-pi4ths), 0,
        sin(pi4ths), cos(-pi4ths), 0,
        0, 0, 1;
    Matrix3f transMat;
    Vector3f transVec(1, 2, 0);
    transMat << 
        1, 0, 1,
        0, 1, 2,
        0, 0, 1;
    
    Matrix3f mat;
    mat <<
        0, 0, 1,
        0, 0, 2,
        0, 0, 0;
    mat += rotMat;
    std::cout << "transVec\n";
    std::cout << transMat.rightCols(1) << std::endl;
    std::cout << "mat\n";
    std::cout << mat << std::endl;
    std::cout << "result of mat*p\n";
    std::cout << mat*p<<std::endl;
    std::cout << "result of rotMat*p + transVec\n";
    std::cout << rotMat * p + transMat.rightCols(1)<<std::endl;
    std::cout << "resutl of transMat*rotMat*p\n";
    std::cout << transMat * rotMat * p<<std::endl;
    return mat * p;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix3f rotation;
    float nx = axis.x(), ny = axis.y(), nz = axis.z();
    float cosa = std::cos(angle), sina = std::sin(angle);
    rotation <<
        0, -nz, ny,
        nz, 0, -nx,
        -ny, nx, 0;
    rotation *= sina;
    rotation += cosa * Eigen::Matrix3f::Identity();
    rotation += (1 - cosa) * axis * axis.transpose();
    std::cout << "get_rotation\n";
    std::cout << rotation << std::endl;
    std::cout << "get_rotation\n";
    Eigen::Matrix4f r4;
    r4.block(0,0,3,3) << rotation;
    r4.col(3) << 0,0,0,1;
    r4.row(3).head(3) << 0, 0, 0;
    std::cout << r4 << std::endl;
    return r4;
}

int main() {
    get_rotation(Vector3f(0, 0, 1), 3.1415926 / 4);
    // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a / b << std::endl;  //0.5
    std::cout << sqrt(b) << std::endl;
    std::cout << acos(-1) << std::endl;    //3.14
    std::cout << sin(30.0 / 180.0 * acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Vector3f v(1.0f, 2.0f, 3.0f);
    Vector3f w(1.0f, 0.0f, 0.0f);
    Vector3f p;
    p = Vector3f(v + w);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    std::cout << p << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;

    // vector dot product
    std::cout << "Example of dot product \n";
    std::cout << v.dot(p) << std::endl;

    // vector cross product
    std::cout << "Example of cross product \n";
    std::cout << v.cross(p) << std::endl;
    std::cout << p.cross(v) << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Matrix3f i, j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    // matrix add i + j
    std::cout << "Example of matrix add \n";
    std::cout << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "Example of matrix scalar mutiply \n";
    std::cout << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "Example of matrix mutiply \n";
    std::cout << i * j << std::endl;
    // matrix multiply vector i * v
    std::cout << "Example of matrix multiply vector \n";
    std::cout << i * v << std::endl;
    std::cout << i.rightCols(1)<<std::endl;
    processVector(Vector3f(2, 1, 1));
    std::cout << "mytest\n";
    std::cout << v * v.transpose()<< std::endl;
    std::cout << v.transpose() * v << std::endl;

    return 0;
}