#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_
#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <utility>
#include "plan_env/sdf_map.hpp"
#include "plan_env/obj_predictor.hpp"
#include "plan_env/environment_interface.hpp"
#include "plan_env/traversability_layer.hpp"

namespace fast_planner
{
    class EDTEnvironment : public sentry_nav::EnvironmentInterface
    {
    private:
        ObjPrediction obj_prediction_;
        ObjScale obj_scale_;
        double resolution_inv_;
        double distToBox(int idx, const Eigen::Vector2d &pos, const double &time);
        double minDistToAllBox(const Eigen::Vector2d &pos, const double &time);

    public:
        EDTEnvironment(/* args */)
        {
        }
        ~EDTEnvironment() override
        {
        }
        SDFMap::Ptr sdf_map_;
        sentry_nav::TraversabilityLayer::Ptr trav_layer_;  // 可选静态标注层 (nullptr=未挂载)
        void init();
        void setMap(SDFMap::Ptr map);
        // 加载静态可通行性标注 (失败则保持未挂载, 查询全部放行)。返回是否成功。
        bool loadTraversability(const std::string &yaml_path)
        {
            auto layer = std::make_shared<sentry_nav::TraversabilityLayer>();
            if (layer->loadFromYaml(yaml_path) && layer->enabled())
            {
                trav_layer_ = layer;
                return true;
            }
            return false;
        }
        sentry_nav::TraversabilityLayer::Ptr getTraversability() const { return trav_layer_; }
        void setObjPrediction(ObjPrediction prediction);
        void setObjScale(ObjScale scale);
        void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
        void interpolateBilinear(double values[2][2], const Eigen::Vector2d &diff,
                                 double &value, Eigen::Vector2d &grad);
        void evaluateEDTWithGrad(const Eigen::Vector2d &pos, double time,
                                 double &dist, Eigen::Vector2d &grad) override;
        double getDistanceAtIndex(const Eigen::Vector2i &idx);
        void evaluateEDTBiquadratic(const Eigen::Vector2d &pos, double &dist,
                                    Eigen::Vector2d &grad);
        double evaluateCoarseEDT(Eigen::Vector2d &pos, double time);
        bool hasDynamicObjects() const override
        {
            return obj_prediction_ != nullptr && !obj_prediction_->empty();
        }
        void getMapRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) override
        {
            sdf_map_->getRegion(ori, size);
        }

        // EnvironmentInterface overrides (delegate to sdf_map_)
        double getDistance(const Eigen::Vector2d &pos) override
        {
            return sdf_map_->getDistance(pos);
        }
        int getInflateOccupancy(const Eigen::Vector2d &pos) override
        {
            return sdf_map_->getInflateOccupancy(const_cast<Eigen::Vector2d&>(pos));
        }
        bool isInMap(const Eigen::Vector2d &pos) override
        {
            return sdf_map_->isInMap(pos);
        }
        double getResolution() override
        {
            return sdf_map_->getResolution();
        }
        // evaluateCoarseEDT override with const ref
        double evaluateCoarseEDT(const Eigen::Vector2d &pos, double time) override;

        // --- 可通行性标注层查询 (delegate to trav_layer_, nullptr-safe) ---
        int getTravType(const Eigen::Vector2d &pos) override
        {
            return trav_layer_ ? (int)trav_layer_->getType(pos) : 0;
        }
        bool isDirectionAllowed(const Eigen::Vector2d &pos, const Eigen::Vector2d &travel_dir) override
        {
            return trav_layer_ ? trav_layer_->isDirectionAllowed(pos, travel_dir) : true;
        }
        bool getOnewayConstraint(const Eigen::Vector2d &pos, Eigen::Vector2d &dir, double &cos_tol) override
        {
            if (trav_layer_) return trav_layer_->getOnewayConstraint(pos, dir, cos_tol);
            dir.setZero();
            cos_tol = -1.0;
            return false;
        }

        typedef shared_ptr<EDTEnvironment> Ptr;
    };
}
#endif