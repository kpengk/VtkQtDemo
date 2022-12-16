#pragma once
#include <vector>

using TrianglePointId = std::array<int, 3>;
using Point3D = std::array<float, 3>;

class Mesh {
public:
    Mesh(float max_dist = 5.0)
        : pre_start_{}
        , cur_start_{}
        , max_dist_{max_dist} {}

    void reserve(std::size_t n) { 
        points_.reserve(n);
        triangles_.reserve(n);
    }

    void clear() {
        pre_start_ = 0;
        cur_start_ = 0;
        points_.clear();
        triangles_.clear();
    }

    void add_section(const std::vector<Point3D>& points) {
        std::copy(points.cbegin(), points.cend(), std::back_insert_iterator(points_));

        make_triangles(pre_start_, cur_start_, cur_start_, points_.size());

        pre_start_ = cur_start_;
        cur_start_ = points_.size();
    }

    const std::vector<Point3D>& points() const { return points_; }
    const std::vector<TrianglePointId>& triangles() const { return triangles_; }

private:
    float squared(float val) { return val * val; }

    float squared_sum(const Point3D& p1, const Point3D& p2) {
        return squared(p1[0] - p2[0]) + squared(p1[1] - p2[1]) + squared(p1[2] - p2[2]);
    }

    void make_triangles(int i, int end_i, int j, int end_j) {
        const float max_dist_squared{max_dist_ * max_dist_};

        if (i >= end_i || j >= end_j) {
            return;
        }

        Point3D pi = points_[i];
        Point3D pj = points_[j];
        while (i < end_i - 1 && j < end_j - 1) {
            const Point3D& next_pi = points_[i + 1];
            const Point3D& next_pj = points_[j + 1];

            const auto dist_i_nj = squared_sum(pi, next_pj);
            const auto dist_j_ni = squared_sum(pj, next_pi);

            if (dist_i_nj < dist_j_ni) {
                if (dist_i_nj < max_dist_squared && dist_j_ni < max_dist_squared) {
                    triangles_.emplace_back(TrianglePointId{j, i, j + 1});
                }
                pj = next_pj;
                ++j;
            } else {
                if (dist_i_nj < max_dist_squared && dist_j_ni < max_dist_squared) {
                    triangles_.emplace_back(TrianglePointId{j, i, i + 1});
                }

                pi = next_pi;
                ++i;
            }
        }
    }

private:
    const float max_dist_;
    std::size_t pre_start_;
    std::size_t cur_start_;
    std::vector<Point3D> points_;
    std::vector<TrianglePointId> triangles_;
};
