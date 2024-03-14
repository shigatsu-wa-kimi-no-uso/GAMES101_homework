#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    int len = control_points.size();
    if (len == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> next_control_points;
    next_control_points.resize(len - 1);
    for (int i = 0; i < len - 1; i++) {
        next_control_points[i] = control_points[i] + t * (control_points[i + 1] - control_points[i]);
    }
    return recursive_bezier(next_control_points,t);

}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        auto point = recursive_bezier(control_points, t);
        int round_pix_x = round(point.x), round_pix_y = round(point.y);
        float dis2_r = pow(round_pix_x - point.x, 2) + pow(round_pix_y - point.y, 2);
        uchar old_val_r = window.at<cv::Vec3b>(round_pix_y, round_pix_x)[1];
        window.at<cv::Vec3b>(round_pix_y, round_pix_x)[1] = std::max(255 * (-dis2_r + 1), (float)old_val_r);
        int trunc_pix_x = point.x;
        int trunc_pix_y = point.y;
        if (trunc_pix_x != round_pix_x || trunc_pix_y != round_pix_y) {
            float dis2_t = pow(trunc_pix_x - point.x, 2) + pow(trunc_pix_y - point.y, 2);
            uchar old_val_t = window.at<cv::Vec3b>(trunc_pix_y, trunc_pix_x)[1];
            window.at<cv::Vec3b>(trunc_pix_y, trunc_pix_x)[1] = std::max(255 * (-dis2_t + 1), (float)old_val_t);// *dis2_t;
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
