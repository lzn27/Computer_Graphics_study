#include <eigen3/Eigen/Eigen>

#include <iostream>

#include <opencv2/opencv.hpp>



#include "Triangle.hpp"

#include "rasterizer.hpp"



constexpr double MY_PI = 3.1415926;



Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {

    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();



    Eigen::Matrix4f translate;

    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2],

        0, 0, 0, 1;



    view = translate * view;



    return view;

}



Eigen::Matrix4f get_model_matrix(float rotation_angle) {

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();



    Eigen::Matrix4f rotate;

    rotate << std::cos(rotation_angle / 180.0 * MY_PI),

        -1.0 * std::sin(rotation_angle / 180.0 * MY_PI), 0.0, 0.0,

        std::sin(rotation_angle / 180.0 * MY_PI),

        std::cos(rotation_angle / 180.0 * MY_PI), 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

        0.0, 0.0, 1.0;

    model = rotate * model;

    // TODO: Implement this function

    // Create the model matrix for rotating the triangle around the Z axis.

    // Then return it.



    return model;

}



Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,

                                      float zNear, float zFar)



{

    // Students will implement this function



    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();



    Eigen::Matrix4f squish;

    squish << zNear, 0.0, 0.0, 0.0, 0.0, zNear, 0.0, 0.0, 0.0, 0.0, zNear + zFar,

        -1.0 * zNear * zFar, 0.0, 0.0, 1.0, 0.0;

    float t = zNear * std::tan(eye_fov / 2.0 / 180.0 * MY_PI);

    float b = -t;

    float r = aspect_ratio * t;

    float l = -r;

    Eigen::Matrix4f translate; // orthographic projection

    translate << 1.0, 0.0, 0.0, -(r + l) / 2.0, 0.0, 1.0, 0.0, -(t + b) / 2.0,

        0.0, 0.0, 1.0, -(zNear + zFar) / 2.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f scale;

    scale << 2.0 / (r - l), 0.0, 0.0, 0.0, 0.0, 2.0 / (t - b), 0.0, 0.0, 0.0, 0.0,

        2.0 / (zNear - zFar), 0.0, 0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4f ortho = scale * translate;

    projection = ortho * squish * projection;

    // TODO: Implement this function

    // Create the projection matrix for the given parameters.

    // Then return it.



    return projection;

}



Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {

    angle = angle / 180.0 * MY_PI;//angle to rad

    Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();



    float z_angle = MY_PI / 2.0;

    if (axis[1] - 0.0 > 0.0000001)

        z_angle = std::atan(axis[0] / axis[1]);



    Eigen::Matrix4f z_rotate;

    z_rotate << std::cos(z_angle), -1.0 * std::sin(z_angle), 0.0, 0.0,

        std::sin(z_angle), std::cos(z_angle), 0.0,

        0.0, 0.0, 0.0, 1.0,

        0.0, 0.0, 0.0, 0.0, 1.0;

    rotation = z_rotate * rotation;



    Vector4f axis_old = {axis[0], axis[1], axis[2], 1.0};

    Vector4f axis_new = z_rotate * axis_old;



    float x_angle = -MY_PI / 2.0;

    if (axis_new[1] - 0.0 > 0.0000001)

        x_angle = -1.0 * std::atan(axis_new[2] / axis_new[1]);



    Eigen::Matrix4f x_rotate;

    x_rotate << 1.0, 0.0, 0.0, 0.0,

        0.0, std::cos(x_angle), -std::sin(x_angle), 0.0,

        0.0, std::sin(x_angle), std::cos(x_angle), 0.0,

        0.0, 0.0, 0.0, 1.0;

    rotation = x_rotate * rotation;



    Eigen::Matrix4f y_rotate;

    y_rotate << std::cos(angle), 0.0, std::sin(angle), 0.0,

        0.0, 1.0, 0.0, 0.0,

        -std::sin(angle), 0.0, std::cos(angle), 0.0,

        0.0, 0.0, 0.0, 1.0;

    rotation = y_rotate * rotation;



    x_angle = -x_angle;

    x_rotate << 1.0, 0.0, 0.0, 0.0,

        0.0, std::cos(x_angle), -std::sin(x_angle), 0.0,

        0.0, std::sin(x_angle), std::cos(x_angle), 0.0,

        0.0, 0.0, 0.0, 1.0;

    rotation = x_rotate * rotation;



    z_angle = -z_angle;

    z_rotate << std::cos(z_angle), -std::sin(z_angle), 0.0, 0.0,

        std::sin(z_angle), std::cos(z_angle), 0.0,

        0.0, 0.0, 0.0, 1.0, 0.0,

        0.0, 0.0, 0.0, 1.0;

    rotation = z_rotate * rotation;



    return rotation;

}



int main(int argc, const char **argv) {

    float angle = 0;

    bool command_line = false;

    std::string filename = "output.png";



    if (argc >= 3) {

        command_line = true;

        angle = std::stof(argv[2]); // -r by default

        if (argc == 4) {

            filename = std::string(argv[3]);

        } else

            return 0;

    }



    rst::rasterizer r(700, 700);



    Eigen::Vector3f eye_pos = {0, 0, 5};



    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};



    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};



    auto pos_id = r.load_positions(pos);

    auto ind_id = r.load_indices(ind);



    int key = 0;

    int frame_count = 0;



    if (command_line) {

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);



        r.set_model(get_model_matrix(angle));

        r.set_view(get_view_matrix(eye_pos));

        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));



        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());

        image.convertTo(image, CV_8UC3, 1.0f);



        cv::imwrite(filename, image);



        return 0;

    }



    while (key != 27) {

        r.clear(rst::Buffers::Color | rst::Buffers::Depth);



        Vector3f axis = {0.0, 0.0, 1.0};

        r.set_model(get_rotation(axis, angle));

        // r.set_model(get_model_matrix(angle));

        r.set_view(get_view_matrix(eye_pos));

        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));



        r.draw(pos_id, ind_id, rst::Primitive::Triangle);



        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());

        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imshow("image", image);

        key = cv::waitKey(10);



        std::cout << "frame count: " << frame_count++ << '\n';



        if (key == 'a') {

            angle += 10;

        } else if (key == 'd') {

            angle -= 10;

        }

    }



    return 0;

}

