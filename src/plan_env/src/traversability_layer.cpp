#include "plan_env/traversability_layer.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

namespace sentry_nav
{

static TravType parseType(const std::string &s)
{
    if (s == "obstacle" || s == "OBSTACLE") return TravType::OBSTACLE;
    if (s == "oneway" || s == "ONEWAY" || s == "one_way") return TravType::ONEWAY;
    return TravType::FREE;
}

bool TraversabilityLayer::loadFromYaml(const std::string &yaml_path)
{
    enabled_ = false;
    regions_.clear();
    grid_.clear();

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(yaml_path);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[TraversabilityLayer] cannot load yaml '" << yaml_path
                  << "': " << e.what() << std::endl;
        return false;
    }

    try
    {
        // ---- map 元信息 (可选; width/height=0 → 由 regions 包围盒自动推导) ----
        // 此处只取 resolution / origin / default_tol; width/height 与 origin 是否声明
        // 在后面的"网格范围"块统一处理 (避免重复读)。
        if (root["map"])
        {
            const auto &m = root["map"];
            if (m["resolution"]) resolution_ = m["resolution"].as<double>();
            if (m["origin"] && m["origin"].size() >= 2)
            {
                origin_(0) = m["origin"][0].as<double>();
                origin_(1) = m["origin"][1].as<double>();
            }
        }
        if (resolution_ <= 1e-6) resolution_ = 0.05;
        resolution_inv_ = 1.0 / resolution_;

        if (root["default_tolerance_deg"])
            default_tol_deg_ = root["default_tolerance_deg"].as<double>();

        // ---- regions ----
        if (root["regions"])
        {
            for (const auto &rn : root["regions"])
            {
                Region reg;
                reg.id = rn["id"] ? rn["id"].as<std::string>() : "";
                reg.type = parseType(rn["type"] ? rn["type"].as<std::string>() : "free");
                reg.dir_deg = rn["direction_deg"] ? rn["direction_deg"].as<double>() : 0.0;
                reg.tol_deg = rn["tolerance_deg"] ? rn["tolerance_deg"].as<double>()
                                                  : default_tol_deg_;
                if (rn["polygon"])
                {
                    for (const auto &v : rn["polygon"])
                    {
                        if (v.size() >= 2)
                            reg.polygon.emplace_back(v[0].as<double>(), v[1].as<double>());
                    }
                }
                if (reg.polygon.size() >= 3)
                    regions_.push_back(std::move(reg));
                else
                    std::cerr << "[TraversabilityLayer] skip region '" << reg.id
                              << "' (<3 polygon vertices)" << std::endl;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "[TraversabilityLayer] yaml parse error: " << e.what() << std::endl;
        return false;
    }

    if (regions_.empty())
    {
        std::cerr << "[TraversabilityLayer] no valid regions in '" << yaml_path
                  << "'; layer disabled." << std::endl;
        return false;
    }

    // ---- 网格范围: 优先用声明值, 否则从 regions 包围盒推导 ----
    {
        int decl_w = 0, decl_h = 0;
        if (root["map"])
        {
            if (root["map"]["width"]) decl_w = root["map"]["width"].as<int>();
            if (root["map"]["height"]) decl_h = root["map"]["height"].as<int>();
        }
        bool have_origin = root["map"] && root["map"]["origin"] &&
                           root["map"]["origin"].size() >= 2;

        if (decl_w > 0 && decl_h > 0 && have_origin)
        {
            width_ = decl_w;
            height_ = decl_h;
        }
        else
        {
            const double inf = std::numeric_limits<double>::infinity();
            Eigen::Vector2d lo(inf, inf), hi(-inf, -inf);
            for (const auto &reg : regions_)
                for (const auto &p : reg.polygon)
                {
                    lo = lo.cwiseMin(p);
                    hi = hi.cwiseMax(p);
                }
            const double margin = 0.5;  // m
            origin_ = lo - Eigen::Vector2d(margin, margin);
            Eigen::Vector2d span = (hi - lo) + Eigen::Vector2d(2 * margin, 2 * margin);
            width_ = std::max(1, (int)std::ceil(span(0) * resolution_inv_));
            height_ = std::max(1, (int)std::ceil(span(1) * resolution_inv_));
        }
    }

    rasterize();
    enabled_ = true;
    std::cerr << "[TraversabilityLayer] loaded " << regions_.size() << " regions, grid "
              << width_ << "x" << height_ << " @ " << resolution_ << "m, origin ("
              << origin_(0) << "," << origin_(1) << ")" << std::endl;
    return true;
}

void TraversabilityLayer::rasterize()
{
    grid_.assign((size_t)width_ * height_, TravCell{});
    // 后写覆盖先写: regions 顺序即优先级 (后者覆盖前者重叠格)
    for (const auto &reg : regions_)
    {
        TravCell cell;
        cell.type = reg.type;
        if (reg.type == TravType::ONEWAY)
        {
            double a = reg.dir_deg * M_PI / 180.0;
            cell.dir = Eigen::Vector2d(std::cos(a), std::sin(a));
            double tol = std::clamp(reg.tol_deg, 0.0, 180.0);
            cell.cos_tol = std::cos(tol * M_PI / 180.0);
        }
        // 仅遍历该 region 包围盒内的格 (而非全图)
        const double inf = std::numeric_limits<double>::infinity();
        Eigen::Vector2d lo(inf, inf), hi(-inf, -inf);
        for (const auto &p : reg.polygon)
        {
            lo = lo.cwiseMin(p);
            hi = hi.cwiseMax(p);
        }
        int gx0, gy0, gx1, gy1;
        worldToGrid(lo, gx0, gy0);
        worldToGrid(hi, gx1, gy1);
        gx0 = std::max(0, gx0); gy0 = std::max(0, gy0);
        gx1 = std::min(width_ - 1, gx1); gy1 = std::min(height_ - 1, gy1);
        for (int gx = gx0; gx <= gx1; ++gx)
            for (int gy = gy0; gy <= gy1; ++gy)
            {
                Eigen::Vector2d c((gx + 0.5) * resolution_ + origin_(0),
                                  (gy + 0.5) * resolution_ + origin_(1));
                if (pointInPolygon(c, reg.polygon))
                    grid_[(size_t)gx * height_ + gy] = cell;
            }
    }
}

bool TraversabilityLayer::worldToGrid(const Eigen::Vector2d &p, int &gx, int &gy) const
{
    gx = (int)std::floor((p(0) - origin_(0)) * resolution_inv_);
    gy = (int)std::floor((p(1) - origin_(1)) * resolution_inv_);
    return inGrid(gx, gy);
}

bool TraversabilityLayer::pointInPolygon(const Eigen::Vector2d &p,
                                         const std::vector<Eigen::Vector2d> &poly)
{
    // 射线投射 (奇偶规则)
    bool inside = false;
    const size_t n = poly.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        const double xi = poly[i](0), yi = poly[i](1);
        const double xj = poly[j](0), yj = poly[j](1);
        bool intersect = ((yi > p(1)) != (yj > p(1))) &&
                         (p(0) < (xj - xi) * (p(1) - yi) / (yj - yi + 1e-12) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

TravType TraversabilityLayer::getType(const Eigen::Vector2d &world_pos) const
{
    if (!enabled_) return TravType::FREE;
    int gx, gy;
    if (!worldToGrid(world_pos, gx, gy)) return TravType::FREE;
    return grid_[(size_t)gx * height_ + gy].type;
}

bool TraversabilityLayer::getOnewayConstraint(const Eigen::Vector2d &world_pos,
                                              Eigen::Vector2d &dir, double &cos_tol) const
{
    dir.setZero();
    cos_tol = -1.0;
    if (!enabled_) return false;
    int gx, gy;
    if (!worldToGrid(world_pos, gx, gy)) return false;
    const TravCell &c = grid_[(size_t)gx * height_ + gy];
    if (c.type != TravType::ONEWAY) return false;
    dir = c.dir;
    cos_tol = c.cos_tol;
    return true;
}

bool TraversabilityLayer::isDirectionAllowed(const Eigen::Vector2d &world_pos,
                                             const Eigen::Vector2d &travel_dir) const
{
    if (!enabled_) return true;
    int gx, gy;
    if (!worldToGrid(world_pos, gx, gy)) return true;
    const TravCell &c = grid_[(size_t)gx * height_ + gy];
    switch (c.type)
    {
    case TravType::FREE:
        return true;
    case TravType::OBSTACLE:
        return false;
    case TravType::ONEWAY:
    {
        double n = travel_dir.norm();
        if (n < 1e-6) return true;  // 无方向信息 (静止) → 放行
        return (travel_dir.dot(c.dir) / n) >= c.cos_tol;
    }
    }
    return true;
}

}  // namespace sentry_nav
