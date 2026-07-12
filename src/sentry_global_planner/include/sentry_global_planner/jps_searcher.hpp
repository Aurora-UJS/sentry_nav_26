#pragma once

#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <sentry_global_planner/global_map.hpp>
#include <plan_env/traversability_layer.hpp>

namespace sentry_global
{

class JPSSearcher
{
public:
    void setMap(const MapInterface *map) { map_ = map; }
    void setParams(double esdf_weight, double safety_dist, double waypoint_spacing,
                   double oneway_penalty = 50.0)
    {
        esdf_weight_ = esdf_weight;
        safety_dist_ = safety_dist;
        waypoint_spacing_ = waypoint_spacing;
        oneway_penalty_ = oneway_penalty;
    }

    // 可通行性标注层 (可选, 外部持有生命周期; 空指针 → 行为与原先完全一致)
    void setTravLayer(const sentry_nav::TraversabilityLayer *t) { trav_ = t; }

    bool search(const Eigen::Vector2d &start_pos, const Eigen::Vector2d &goal_pos,
                std::vector<Eigen::Vector2d> &path)
    {
        path.clear();
        auto s = map_->worldToGrid(start_pos);
        auto g = map_->worldToGrid(goal_pos);

        if (!map_->isInMap(s(0), s(1)) || !map_->isInMap(g(0), g(1)))
            return false;
        if (map_->isOccupied(g(0), g(1)))
            return false;

        // A* with JPS pruning
        open_ = {};
        closed_.clear();
        parent_.clear();
        g_score_.clear();

        int sid = toId(s(0), s(1));
        g_score_[sid] = 0.0;
        open_.push({sid, heuristic(s(0), s(1), g(0), g(1))});

        while (!open_.empty())
        {
            auto cur = open_.top();
            open_.pop();

            if (closed_.count(cur.id))
                continue;
            closed_.insert(cur.id);

            int cx, cy;
            fromId(cur.id, cx, cy);

            if (cx == g(0) && cy == g(1))
            {
                // Reconstruct path
                reconstructPath(cur.id, start_pos, goal_pos, path);
                simplifyPath(path);
                return true;
            }

            // Get successors via JPS
            auto successors = findSuccessors(cx, cy, g(0), g(1));
            for (auto &[nx, ny] : successors)
            {
                int nid = toId(nx, ny);
                if (closed_.count(nid))
                    continue;

                double step = std::hypot(nx - cx, ny - cy) * map_->resolution();
                double esdf_val = map_->getESDF(nx, ny);
                double cost_mult = 1.0;
                if (esdf_val < safety_dist_ * 3.0 && esdf_val > 0)
                    cost_mult += esdf_weight_ / (esdf_val + 0.1);

                // 可通行性标注层 ONEWAY 软门控: cur→successor 这一段若逆着单向箭头
                // 穿过 oneway 区, 则放大代价 (软约束, 绝不硬跳过, 以保持 JPS 完整/连通;
                // 硬性方向约束由局部规划器 A*/MINCO 执行)。沿段采样若干点判定。
                if (trav_ && trav_->enabled())
                {
                    Eigen::Vector2d wc = map_->gridToWorld(cx, cy);
                    Eigen::Vector2d wn = map_->gridToWorld(nx, ny);
                    Eigen::Vector2d travel = wn - wc;  // 世界系行进方向 (长度无关, 层内归一化)
                    const int n_sample = 5;
                    for (int si = 0; si <= n_sample; ++si)
                    {
                        double t = (double)si / (double)n_sample;
                        Eigen::Vector2d pt = wc + t * travel;
                        if (!trav_->isDirectionAllowed(pt, travel))
                        {
                            cost_mult += oneway_penalty_;
                            break;
                        }
                    }
                }

                double new_g = g_score_[cur.id] + step * cost_mult;
                if (!g_score_.count(nid) || new_g < g_score_[nid])
                {
                    g_score_[nid] = new_g;
                    parent_[nid] = cur.id;
                    double f = new_g + heuristic(nx, ny, g(0), g(1));
                    open_.push({nid, f});
                }
            }
        }
        return false;
    }

private:
    const MapInterface *map_ = nullptr;
    const sentry_nav::TraversabilityLayer *trav_ = nullptr;  // 可选标注层 (外部持有生命周期)
    double esdf_weight_ = 2.0;
    double safety_dist_ = 0.3;
    double waypoint_spacing_ = 1.0;
    double oneway_penalty_ = 50.0;  // ONEWAY 逆向软惩罚: cost_mult 增量

    struct OpenNode {
        int id;
        double f;
        bool operator>(const OpenNode &o) const { return f > o.f; }
    };

    std::priority_queue<OpenNode, std::vector<OpenNode>, std::greater<OpenNode>> open_;
    std::unordered_set<int> closed_;
    std::unordered_map<int, int> parent_;
    std::unordered_map<int, double> g_score_;

    int toId(int x, int y) const { return x * 100000 + y; }
    void fromId(int id, int &x, int &y) const { x = id / 100000; y = id % 100000; }

    double heuristic(int x1, int y1, int x2, int y2) const
    {
        return std::hypot(x2 - x1, y2 - y1) * map_->resolution();
    }

    bool isFree(int x, int y) const
    {
        if (!map_->isInMap(x, y)) return false;
        if (map_->isOccupied(x, y)) return false;
        if (map_->getESDF(x, y) < safety_dist_) return false;
        // 可通行性标注层 OBSTACLE 叠加: 人工补充障碍 (标注只能"增加"障碍, 不放宽既有判定)
        if (trav_ && trav_->enabled() &&
            trav_->getType(map_->gridToWorld(x, y)) == sentry_nav::TravType::OBSTACLE)
            return false;
        return true;
    }

    // JPS: jump in direction (dx,dy) from (x,y), return jump point or (-1,-1)
    Eigen::Vector2i jump(int x, int y, int dx, int dy, int gx, int gy) const
    {
        int nx = x + dx, ny = y + dy;
        if (!isFree(nx, ny))
            return {-1, -1};

        if (nx == gx && ny == gy)
            return {nx, ny};

        // Diagonal
        if (dx != 0 && dy != 0)
        {
            // Forced neighbors check
            if ((!isFree(nx - dx, ny) && isFree(nx - dx, ny + dy)) ||
                (!isFree(nx, ny - dy) && isFree(nx + dx, ny - dy)))
                return {nx, ny};

            // Recurse cardinal directions
            if (jump(nx, ny, dx, 0, gx, gy)(0) != -1 ||
                jump(nx, ny, 0, dy, gx, gy)(0) != -1)
                return {nx, ny};
        }
        else
        {
            // Cardinal
            if (dx != 0)
            {
                if ((!isFree(nx, ny + 1) && isFree(nx + dx, ny + 1)) ||
                    (!isFree(nx, ny - 1) && isFree(nx + dx, ny - 1)))
                    return {nx, ny};
            }
            else
            {
                if ((!isFree(nx + 1, ny) && isFree(nx + 1, ny + dy)) ||
                    (!isFree(nx - 1, ny) && isFree(nx - 1, ny + dy)))
                    return {nx, ny};
            }
        }

        return jump(nx, ny, dx, dy, gx, gy);
    }

    std::vector<std::pair<int, int>> findSuccessors(int x, int y, int gx, int gy)
    {
        std::vector<std::pair<int, int>> result;

        // Get parent direction
        int px = 0, py = 0;
        int id = toId(x, y);
        if (parent_.count(id))
        {
            int par_x, par_y;
            fromId(parent_[id], par_x, par_y);
            px = (x == par_x) ? 0 : (x > par_x ? 1 : -1);
            py = (y == par_y) ? 0 : (y > par_y ? 1 : -1);
        }

        // If no parent (start node), try all 8 directions
        std::vector<std::pair<int, int>> dirs;
        if (px == 0 && py == 0)
        {
            for (int dx = -1; dx <= 1; ++dx)
                for (int dy = -1; dy <= 1; ++dy)
                    if (dx != 0 || dy != 0)
                        dirs.push_back({dx, dy});
        }
        else
        {
            // JPS pruned neighbors
            if (px != 0 && py != 0)
            {
                // Diagonal: natural neighbors
                dirs.push_back({px, py});
                dirs.push_back({px, 0});
                dirs.push_back({0, py});
                // Forced neighbors
                if (!isFree(x - px, y)) dirs.push_back({-px, py});
                if (!isFree(x, y - py)) dirs.push_back({px, -py});
            }
            else if (px != 0)
            {
                dirs.push_back({px, 0});
                if (!isFree(x, y + 1)) dirs.push_back({px, 1});
                if (!isFree(x, y - 1)) dirs.push_back({px, -1});
            }
            else
            {
                dirs.push_back({0, py});
                if (!isFree(x + 1, y)) dirs.push_back({1, py});
                if (!isFree(x - 1, y)) dirs.push_back({-1, py});
            }
        }

        for (auto &[dx, dy] : dirs)
        {
            auto jp = jump(x, y, dx, dy, gx, gy);
            if (jp(0) != -1)
                result.push_back({jp(0), jp(1)});
        }
        return result;
    }

    void reconstructPath(int goal_id, const Eigen::Vector2d &start_pos,
                         const Eigen::Vector2d &goal_pos,
                         std::vector<Eigen::Vector2d> &path)
    {
        std::vector<int> ids;
        int cur = goal_id;
        while (parent_.count(cur))
        {
            ids.push_back(cur);
            cur = parent_[cur];
        }
        ids.push_back(cur); // start
        std::reverse(ids.begin(), ids.end());

        path.push_back(start_pos);
        for (size_t i = 1; i + 1 < ids.size(); ++i)
        {
            int x, y;
            fromId(ids[i], x, y);
            path.push_back(map_->gridToWorld(x, y));
        }
        path.push_back(goal_pos);
    }

    void simplifyPath(std::vector<Eigen::Vector2d> &path)
    {
        if (path.size() <= 2) return;

        std::vector<Eigen::Vector2d> simplified;
        simplified.push_back(path.front());

        size_t i = 0;
        while (i < path.size() - 1)
        {
            size_t farthest = i + 1;
            for (size_t j = i + 2; j < path.size(); ++j)
            {
                if (hasLineOfSight(path[i], path[j]))
                    farthest = j;
                else
                    break;
            }
            simplified.push_back(path[farthest]);
            i = farthest;
        }

        // Enforce minimum waypoint spacing.
        // 注意: 只有当"跳过本点后新连线仍有安全视线"时才允许丢弃 —
        // 原实现按间距无条件丢角点，产生从未检查过的切角捷径
        // (实测输出过沿线 0 间隙的路径，机器人跟着它卡死在夹道里)。
        std::vector<Eigen::Vector2d> spaced;
        spaced.push_back(simplified.front());
        for (size_t k = 1; k + 1 < simplified.size(); ++k)
        {
            if ((simplified[k] - spaced.back()).norm() >= waypoint_spacing_ ||
                !hasLineOfSight(spaced.back(), simplified[k + 1]))
                spaced.push_back(simplified[k]);
        }
        spaced.push_back(simplified.back());
        path = spaced;
    }

    bool hasLineOfSight(const Eigen::Vector2d &a, const Eigen::Vector2d &b) const
    {
        auto ga = map_->worldToGrid(a);
        auto gb = map_->worldToGrid(b);
        int x0 = ga(0), y0 = ga(1), x1 = gb(0), y1 = gb(1);
        int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        while (true)
        {
            if (!isFree(x0, y0)) return false;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 <  dx) { err += dx; y0 += sy; }
        }
        return true;
    }
};

} // namespace sentry_global
