#include <iostream>
#include <opencv2/opencv.hpp>
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
    Eigen::Vector4f normal_interpolated = get_bary_interpolated_value(payload.bar, payload.view_normals);
    Eigen::Vector3f return_color = (normal_interpolated.head(3).normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
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

    auto texture_path = "hmap.jpg";
    Texture texture = Texture(obj_path + texture_path);

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = normal_fragment_shader;

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
