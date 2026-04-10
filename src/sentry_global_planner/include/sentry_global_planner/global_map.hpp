#pragma once

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <limits>
#include <algorithm>
#include <plan_env/sdf_map.hpp>

namespace sentry_global
{

// ============================================================
// MapInterface: JPS 只依赖这个抽象接口
// ============================================================
class MapInterface
{
public:
    virtual ~MapInterface() = default;
    virtual bool isOccupied(int x, int y) const = 0;
    virtual double getESDF(int x, int y) const = 0;
    virtual double getESDF(const Eigen::Vector2d &pos) const = 0;
    virtual Eigen::Vector2i worldToGrid(const Eigen::Vector2d &pos) const = 0;
    virtual Eigen::Vector2d gridToWorld(int x, int y) const = 0;
    virtual bool isInMap(int x, int y) const = 0;
    virtual int width() const = 0;
    virtual int height() const = 0;
    virtual double resolution() const = 0;
};

// ============================================================
// PriorMap: 先验 PGM 地图 + 静态 Felzenszwalb ESDF
// ============================================================
class PriorMap : public MapInterface
{
public:
    void loadFromYaml(const std::string &yaml_path)
    {
        // 解析 ROS2 nav2 格式 YAML (手动解析, 不依赖 yaml-cpp)
        std::ifstream ifs(yaml_path);
        if (!ifs.is_open())
            throw std::runtime_error("Cannot open map yaml: " + yaml_path);

        std::string line, image_file;
        double occ_thresh = 0.65;
        while (std::getline(ifs, line)) {
            if (line.find("image:") != std::string::npos)
                image_file = line.substr(line.find(':') + 2);
            else if (line.find("resolution:") != std::string::npos)
                resolution_ = std::stod(line.substr(line.find(':') + 2));
            else if (line.find("origin:") != std::string::npos) {
                auto bracket = line.find('[');
                auto comma1 = line.find(',', bracket);
                auto comma2 = line.find(',', comma1 + 1);
                auto bracket2 = line.find(']');
                origin_(0) = std::stod(line.substr(bracket + 1, comma1 - bracket - 1));
                origin_(1) = std::stod(line.substr(comma1 + 1, comma2 - comma1 - 1));
            }
            else if (line.find("occupied_thresh:") != std::string::npos)
                occ_thresh = std::stod(line.substr(line.find(':') + 2));
        }
        resolution_inv_ = 1.0 / resolution_;

        // 图片路径: 相对于 yaml 所在目录
        std::string dir = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
        // 去除 image_file 首尾空白
        while (!image_file.empty() && (image_file.back() == ' ' || image_file.back() == '\r'))
            image_file.pop_back();
        std::string pgm_path = dir + image_file;

        cv::Mat img = cv::imread(pgm_path, cv::IMREAD_GRAYSCALE);
        if (img.empty())
            throw std::runtime_error("Cannot load map image: " + pgm_path);

        // PGM: width=cols, height=rows
        // 注意: ROS2 地图 Y 轴向上, PGM 行 0 在顶部 → 需要 flip
        cv::flip(img, img, 0);

        width_ = img.cols;
        height_ = img.rows;
        int sz = width_ * height_;
        occupancy_.resize(sz, 0);

        // ROS2 nav2 trinary map format:
        // pixel ≈ 0    (black)  = occupied
        // pixel ≈ 254  (white)  = free
        // pixel ≈ 205  (gray)   = unknown → treat as occupied (conservative)
        //
        // Use occupied_thresh to determine cutoff:
        // occupied: pixel < 255 * (1 - occ_thresh)
        // free:     pixel > 255 * (1 - occ_thresh)  (white pixels only)
        // This ensures unknown (205) is treated as occupied since 205 < 255*(1-0.65)=89? No...
        //
        // Simple trinary approach: only pure white (>250) is free
        for (int r = 0; r < height_; ++r)
            for (int c = 0; c < width_; ++c) {
                uint8_t val = img.at<uint8_t>(r, c);
                // Only bright white pixels are free; black + gray (unknown) = occupied
                occupancy_[c * height_ + r] = (val > 250) ? 0 : 1;
            }
    }

    void computeESDF()
    {
        int sz = width_ * height_;
        esdf_.resize(sz, 0.0);
        std::vector<double> tmp(sz, 0.0);

        // Felzenszwalb 1D distance transform helper
        auto fillDT = [](auto f_get, auto f_set, int n) {
            std::vector<int> v(n);
            std::vector<double> z(n + 1);
            int k = 0;
            v[0] = 0;
            z[0] = -1e18;
            z[1] = 1e18;
            for (int q = 1; q < n; ++q) {
                double s;
                do {
                    s = ((f_get(q) + (double)q * q) - (f_get(v[k]) + (double)v[k] * v[k])) / (2.0 * q - 2.0 * v[k]);
                    if (s <= z[k]) k--;
                    else break;
                } while (true);
                k++;
                v[k] = q;
                z[k] = s;
                z[k + 1] = 1e18;
            }
            k = 0;
            for (int q = 0; q < n; ++q) {
                while (z[k + 1] < q) k++;
                f_set(q, (q - v[k]) * (q - v[k]) + f_get(v[k]));
            }
        };

        // Positive DT (distance to nearest occupied)
        // Pass 1: along Y for each X
        for (int x = 0; x < width_; ++x) {
            fillDT(
                [&](int y) -> double { return occupancy_[x * height_ + y] ? 0.0 : 1e18; },
                [&](int y, double val) { tmp[x * height_ + y] = val; },
                height_);
        }
        // Pass 2: along X for each Y
        for (int y = 0; y < height_; ++y) {
            fillDT(
                [&](int x) -> double { return tmp[x * height_ + y]; },
                [&](int x, double val) { esdf_[x * height_ + y] = std::sqrt(val) * resolution_; },
                width_);
        }

        // Negative DT (distance inside obstacles)
        std::vector<double> neg_esdf(sz, 0.0);
        for (int x = 0; x < width_; ++x) {
            fillDT(
                [&](int y) -> double { return occupancy_[x * height_ + y] ? 1e18 : 0.0; },
                [&](int y, double val) { tmp[x * height_ + y] = val; },
                height_);
        }
        for (int y = 0; y < height_; ++y) {
            fillDT(
                [&](int x) -> double { return tmp[x * height_ + y]; },
                [&](int x, double val) { neg_esdf[x * height_ + y] = std::sqrt(val) * resolution_; },
                width_);
        }

        // Combine: positive outside, negative inside
        for (int i = 0; i < sz; ++i) {
            if (neg_esdf[i] > 0.0)
                esdf_[i] += (-neg_esdf[i] + resolution_);
        }
    }

    // --- MapInterface ---
    bool isOccupied(int x, int y) const override
    {
        if (!isInMap(x, y)) return true;
        return occupancy_[x * height_ + y] != 0;
    }

    double getESDF(int x, int y) const override
    {
        if (!isInMap(x, y)) return -1.0;
        return esdf_[x * height_ + y];
    }

    double getESDF(const Eigen::Vector2d &pos) const override
    {
        Eigen::Vector2i idx = worldToGrid(pos);
        if (!isInMap(idx(0), idx(1))) return -1.0;
        return esdf_[idx(0) * height_ + idx(1)];
    }

    Eigen::Vector2i worldToGrid(const Eigen::Vector2d &pos) const override
    {
        return Eigen::Vector2i(
            (int)std::floor((pos(0) - origin_(0)) * resolution_inv_),
            (int)std::floor((pos(1) - origin_(1)) * resolution_inv_));
    }

    Eigen::Vector2d gridToWorld(int x, int y) const override
    {
        return Eigen::Vector2d(
            (x + 0.5) * resolution_ + origin_(0),
            (y + 0.5) * resolution_ + origin_(1));
    }

    bool isInMap(int x, int y) const override
    {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    int width() const override { return width_; }
    int height() const override { return height_; }
    double resolution() const override { return resolution_; }

private:
    std::vector<uint8_t> occupancy_;
    std::vector<double> esdf_;
    int width_ = 0, height_ = 0;
    double resolution_ = 0.05, resolution_inv_ = 20.0;
    Eigen::Vector2d origin_ = Eigen::Vector2d::Zero();
};

// ============================================================
// OnlineMapProxy: 代理到 plan_env 的 SDFMap (在线模式)
// ============================================================
class OnlineMapProxy : public MapInterface
{
public:
    void setSDFMap(SDFMap::Ptr sdf_map)
    {
        sdf_map_ = sdf_map;
        res_ = sdf_map_->getResolution();
        res_inv_ = 1.0 / res_;
    }

    bool isOccupied(int x, int y) const override
    {
        Eigen::Vector2d pos = gridToWorld(x, y);
        return sdf_map_->getInflateOccupancy(pos) == 1;
    }

    double getESDF(int x, int y) const override
    {
        Eigen::Vector2d pos = gridToWorld(x, y);
        return sdf_map_->getDistance(pos);
    }

    double getESDF(const Eigen::Vector2d &pos) const override
    {
        return sdf_map_->getDistance(pos);
    }

    Eigen::Vector2i worldToGrid(const Eigen::Vector2d &pos) const override
    {
        Eigen::Vector2i id;
        sdf_map_->posToIndex(pos, id);
        return id;
    }

    Eigen::Vector2d gridToWorld(int x, int y) const override
    {
        Eigen::Vector2i id(x, y);
        Eigen::Vector2d pos;
        sdf_map_->indexToPos(id, pos);
        return pos;
    }

    bool isInMap(int x, int y) const override
    {
        return sdf_map_->isInMap(Eigen::Vector2i(x, y));
    }

    int width() const override
    {
        Eigen::Vector2d ori, sz;
        sdf_map_->getRegion(ori, sz);
        return (int)(sz(0) * res_inv_);
    }

    int height() const override
    {
        Eigen::Vector2d ori, sz;
        sdf_map_->getRegion(ori, sz);
        return (int)(sz(1) * res_inv_);
    }

    double resolution() const override { return res_; }

private:
    SDFMap::Ptr sdf_map_;
    double res_ = 0.05, res_inv_ = 20.0;
};

} // namespace sentry_global
