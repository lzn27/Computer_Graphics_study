//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <vector>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions) {
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices) {
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols) {
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f &v3, float w = 1.0f) {
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float i, float j, const Triangle &t) {
    bool flag = true;
    Vector3f p0 = t.v[0], p1 = t.v[1], p2 = t.v[2];
    float ax = i - p0.x(), ay = j - p0.y(), bx = p1.x() - p0.x(), by = p1.y() - p0.y();
    if (ax * by - ay * bx > 0)
        flag = true;
    else
        flag = false;

    if ((i - p1.x()) * (p2.y() - p1.y()) - (j - p1.y()) * (p2.x() - p1.x()) > 0 != flag)
        return false;
    if ((i - p2.x()) * (p0.y() - p2.y()) - (j - p2.y()) * (p0.x() - p2.x()) > 0 != flag)
        return false;

    return true;
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f *v) {
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return {c1, c2, c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type) {
    auto &buf = pos_buf[pos_buffer.pos_id];
    auto &ind = ind_buf[ind_buffer.ind_id];
    auto &col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto &i : ind) {
        Triangle t;
        Eigen::Vector4f v[] = {
            mvp * to_vec4(buf[i[0]], 1.0f),
            mvp * to_vec4(buf[i[1]], 1.0f),
            mvp * to_vec4(buf[i[2]], 1.0f)};
        //Homogeneous division
        for (auto &vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto &vert : v) {
            vert.x() = 0.5 * width * (vert.x() + 1.0);
            vert.y() = 0.5 * height * (vert.y() + 1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i) {
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
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle &t) {
    auto v = t.toVector4();
    float left = std::min(v[0][0], std::min(v[1][0], v[2][0]));
    float bottom = std::min(v[0][1], std::min(v[1][1], v[2][1]));
    float top = std::max(v[0][1], std::max(v[1][1], v[2][1]));
    float right = std::max(v[0][0], std::max(v[1][0], v[2][0]));
    // ----------single pixel sample
    // for (int x = left; x < right + 1; x++) {
    //     for (int y = bottom; y < top + 1; y++) {

    //         if (insideTriangle(x, y, t)) {
    //             auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;
    //             if (z_interpolated < depth_buf[get_index(x, y)]) {
    //                 depth_buf[get_index(x, y)] = z_interpolated;
    //                 set_pixel({x, y, 0}, t.getColor());
    //             }
    //         }
    //     }
    // }
    // ----------super sample
    for (int x = left; x < right + 1; x++) {
        for (int y = bottom; y < top + 1; y++) {
            std::vector<std::array<float, 2>> sample_s;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    sample_s.push_back({x - 0.375f + 0.25f * i, y - 0.375f + 0.25f * j});
                }
            }
            for (int i = 0; i < 16; i++) {
                if (insideTriangle(sample_s[i][0], sample_s[i][1], t)) {
                    auto [alpha, beta, gamma] = computeBarycentric2D(sample_s[i][0], sample_s[i][1], t.v);
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated < depth_buf_s[get_index(x, y)].depth[i]) {
                        depth_buf_s[get_index(x, y)].depth[i] = z_interpolated;
                        // frame_buf[get_index(x, y)] += t.getColor() / 16;
                        depth_buf_s[get_index(x, y)].frame[i] = t.getColor();
                    }
                }
            }
            Vector3f color{0, 0, 0};
            for (int i = 0; i < 16; i++) {
                color += (depth_buf_s[get_index(x, y)].frame[i] / 16);
            }
            set_pixel({x, y, 0}, color);
            // set_pixel({x, y, 0}, (depth_buf_s[get_index(x, y)].frame[0] + depth_buf_s[get_index(x, y)].frame[1] + depth_buf_s[get_index(x, y)].frame[2] + depth_buf_s[get_index(x, y)].frame[3]) / 4);
        }
    }

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f &m) {
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f &v) {
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f &p) {
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color) {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth) {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h) {
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    //--------super sample
    depth_buf_s.resize(w * h);
    //--------
}

int rst::rasterizer::get_index(int x, int y) {
    return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f &point, const Eigen::Vector3f &color) {
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height - 1 - point.y()) * width + point.x();
    frame_buf[ind] = color;
}
