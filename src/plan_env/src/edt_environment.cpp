#include <plan_env/edt_environment.hpp>

namespace fast_planner
{
    void EDTEnvironment::init()
    {
    }
    void EDTEnvironment::setMap(shared_ptr<SDFMap> map)
    {
        this->sdf_map_ = map;
        resolution_inv_ = 1 / sdf_map_->getResolution();
    }

    void EDTEnvironment::setObjPrediction(ObjPrediction prediction)
    {
        this->obj_prediction_ = prediction;
    }

    void EDTEnvironment::setObjScale(ObjScale scale)
    {
        this->obj_scale_ = scale;
    }

    double EDTEnvironment::distToBox(int idx, const Eigen::Vector2d &pos, const double &time)
    {
        // Eigen::Vector3d pos_box = obj_prediction_->at(idx).evaluate(time);
        Eigen::Vector2d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);

        Eigen::Vector2d box_max = pos_box + 0.5 * obj_scale_->at(idx);
        Eigen::Vector2d box_min = pos_box - 0.5 * obj_scale_->at(idx);

        Eigen::Vector2d dist;

        for (int i = 0; i < 2; i++)
        {
            dist(i) = pos(i) >= box_min(i) && pos(i) <= box_max(i) ? 0.0 : min(fabs(pos(i) - box_min(i)), fabs(pos(i) - box_max(i)));
        }

        return dist.norm();
    }

    double EDTEnvironment::minDistToAllBox(const Eigen::Vector2d &pos, const double &time)
    {
        double dist = std::numeric_limits<double>::max();
        for (int i = 0; i < obj_prediction_->size(); i++)
        {
            double di = distToBox(i, pos, time);
            if (di < dist)
                dist = di;
        }

        return dist;
    }

    double EDTEnvironment::distToBoxGrad(int idx, const Eigen::Vector2d &pos,
                                         const double &time, Eigen::Vector2d &grad)
    {
        Eigen::Vector2d pos_box = obj_prediction_->at(idx).evaluateConstVel(time);
        Eigen::Vector2d box_max = pos_box + 0.5 * obj_scale_->at(idx);
        Eigen::Vector2d box_min = pos_box - 0.5 * obj_scale_->at(idx);

        // 带符号位移：该轴在盒内为 0，盒外指向最近面（负=下边界外，正=上边界外）。
        // dist = ||d|| 是到盒的欧氏距离；解析梯度 = d/||d||（单位向量，背离盒面）。
        Eigen::Vector2d d;
        for (int i = 0; i < 2; i++)
        {
            if (pos(i) < box_min(i))       d(i) = pos(i) - box_min(i);
            else if (pos(i) > box_max(i))  d(i) = pos(i) - box_max(i);
            else                           d(i) = 0.0;
        }
        double dist = d.norm();
        if (dist > 1e-12)
            grad = d / dist;   // 单位向量，背离盒面
        else
            grad.setZero();    // 盒内/贴面：无梯度方向，置零（与中心差分一致）
        return dist;
    }

    double EDTEnvironment::minDistToAllBoxWithGrad(const Eigen::Vector2d &pos,
                                                   const double &time, Eigen::Vector2d &grad)
    {
        double best = std::numeric_limits<double>::max();
        grad.setZero();
        for (int i = 0; i < obj_prediction_->size(); i++)
        {
            Eigen::Vector2d g;
            double di = distToBoxGrad(i, pos, time, g);
            if (di < best)
            {
                best = di;
                grad = g;
            }
        }
        return best;
    }

    void EDTEnvironment::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2])
    {
        for (int x = 0; x < 2; x++)
        {
            for (int y = 0; y < 2; y++)
            {

                dists[x][y] = sdf_map_->getDistance(pts[x][y]);
            }
        }
    }

    void EDTEnvironment::interpolateBilinear(double values[2][2],
                                             const Eigen::Vector2d &diff,
                                             double &value,
                                             Eigen::Vector2d &grad)
    {
        // bilinear interpolation
        double v0 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
        double v1 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];

        value = (1 - diff(1)) * v0 + diff(1) * v1;

        // Gradient calculation
        grad[1] = (v1 - v0) * resolution_inv_;
        grad[0] = (1 - diff(1)) * (values[1][0] - values[0][0]) + diff(1) * (values[1][1] - values[0][1]);
        grad[0] *= resolution_inv_;
    }

    void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector2d &pos,
                                             double time, double &dist,
                                             Eigen::Vector2d &grad)
    {
        evaluateEDTBiquadratic(pos, dist, grad);
    }

    double EDTEnvironment::getDistanceAtIndex(const Eigen::Vector2i &idx)
    {
        return sdf_map_->getDistanceByIndex(idx);
    }

    void EDTEnvironment::evaluateEDTBiquadratic(const Eigen::Vector2d &pos,
                                                double &dist, Eigen::Vector2d &grad)
    {
        // 将 pos 转换为网格坐标 (连续)
        Eigen::Vector2d pos_g = (pos - sdf_map_->getMapOrigin()) * resolution_inv_;
        // 最近网格中心索引
        int ix = (int)std::floor(pos_g(0));
        int iy = (int)std::floor(pos_g(1));
        double fx = pos_g(0) - ix;  // [0, 1) 小数部分
        double fy = pos_g(1) - iy;

        // 3x3 邻域: (ix-1..ix+1) x (iy-1..iy+1)
        double d[3][3];
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
            {
                Eigen::Vector2i idx(ix + dx, iy + dy);
                d[dx+1][dy+1] = getDistanceAtIndex(idx);
            }

        // 二次插值基函数: B_{-1}(t)=0.5*(t-0.5)^2, B_0(t)=0.75-(t)^2, B_1(t)=0.5*(t+0.5)^2
        // 等价于用 t = fx-0.5 的二次 B-spline 基
        double tx = fx - 0.5, ty = fy - 0.5;
        double wx[3] = { 0.5*(0.5 - tx)*(0.5 - tx), 0.75 - tx*tx, 0.5*(0.5 + tx)*(0.5 + tx) };
        double wy[3] = { 0.5*(0.5 - ty)*(0.5 - ty), 0.75 - ty*ty, 0.5*(0.5 + ty)*(0.5 + ty) };
        // 基函数导数
        double dwx[3] = { -(0.5 - tx), -2.0*tx, (0.5 + tx) };
        double dwy[3] = { -(0.5 - ty), -2.0*ty, (0.5 + ty) };

        dist = 0.0;
        grad.setZero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            {
                dist += wx[i] * wy[j] * d[i][j];
                grad(0) += dwx[i] * wy[j] * d[i][j];
                grad(1) += wx[i] * dwy[j] * d[i][j];
            }
        grad *= resolution_inv_;
    }

    double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector2d &pos, double time)
    {
        double d1 = sdf_map_->getDistance(pos);
        if (time < 0.0)
        {
            return d1;
        }
        else
        {
            double d2 = minDistToAllBox(pos, time);
            return min(d1, d2);
        }
    }

    double EDTEnvironment::evaluateCoarseEDT(const Eigen::Vector2d &pos, double time)
    {
        Eigen::Vector2d pos_copy = pos;
        return evaluateCoarseEDT(pos_copy, time);
    }

    void EDTEnvironment::evaluateCoarseEDTWithGrad(const Eigen::Vector2d &pos, double time,
                                                   double &dist, Eigen::Vector2d &grad)
    {
        // 静态 ESDF 用双二次插值 + 解析梯度（与 evaluateEDTWithGrad / 静态分支一致）。
        // 原动态分支用 sdf_map_->getDistance(最近格心，无梯度) + 中心差分，梯度几乎恒 0、
        // 只在格边界跳变；这里换成双二次解析梯度，动态障碍盒部分也用解析梯度。
        evaluateEDTBiquadratic(pos, dist, grad);
        if (time >= 0.0 && hasDynamicObjects())
        {
            Eigen::Vector2d dgrad;
            double d2 = minDistToAllBoxWithGrad(pos, time, dgrad);
            if (d2 < dist)
            {
                dist = d2;
                grad = dgrad;
            }
        }
    }

    // EDTEnvironment::
}