#include <path_searching/kinodynamic_astar.hpp>
#include <path_searching/polynomial_solver.hpp>
#include <chrono>
#include <limits>
using namespace std;
namespace fast_planner
{
    KinodynamicAstar::~KinodynamicAstar()
    {
        for (int i = 0; i < allocate_num_; i++)
        {
            delete path_node_pool_[i];
        }
    }
    void KinodynamicAstar::init()
    {
        /* ---------- map params ---------- */
        this->inv_resolution_ = 1.0 / resolution_;
        inv_time_resolution_ = 1.0 / time_resolution_;
        env_->getMapRegion(origin_, map_size_3d_);

        RCLCPP_INFO(node_->get_logger(), "A* init: origin=(%.2f,%.2f), map_size=(%.2f,%.2f)",
                    origin_(0), origin_(1), map_size_3d_(0), map_size_3d_(1));

        /* ---------- pre-allocated node ---------- */
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; i++)
        {
            path_node_pool_[i] = new PathNode;
        }

        phi_ = Eigen::MatrixXd::Identity(4, 4);
        use_node_num_ = 0;
        iter_num_ = 0;
    }
    void KinodynamicAstar::setParam(std::shared_ptr<rclcpp::Node> nh)
    {
        this->node_ = nh;

        max_tau_ = node_->declare_parameter<double>("search.max_tau", 0.6);
        init_max_tau_ = node_->declare_parameter<double>("search.init_max_tau", 0.8);
        max_vel_ = node_->declare_parameter<double>("search.max_vel", 3.0);
        max_acc_ = node_->declare_parameter<double>("search.max_acc", 1.0);
        w_time_ = node_->declare_parameter<double>("search.w_time", 5.0);
        horizon_ = node_->declare_parameter<double>("search.horizon", 100.0);
        resolution_ = node_->declare_parameter<double>("search.resolution_astar", 0.01);
        time_resolution_ = node_->declare_parameter<double>("search.time_resolution", 0.8);
        lambda_heu_ = node_->declare_parameter<double>("search.lambda_heu", 5.0);
        allocate_num_ = node_->declare_parameter<int>("search.allocate_num", 100000);
        check_num_ = node_->declare_parameter<int>("search.check_num", 50);
        robot_radius_ = node_->declare_parameter<double>("search.robot_radius", 0.3);
        w_clearance_ = node_->declare_parameter<double>("search.w_clearance", 20.0);
        start_ignore_radius_ = node_->declare_parameter<double>("search.start_ignore_radius", 0.35);
        clearance_dist_ = node_->declare_parameter<double>("search.clearance_dist", 0.5);
        accept_clearance_ = node_->declare_parameter<double>("search.accept_clearance", 0.28);
        max_search_time_ms_ = node_->declare_parameter<double>("search.max_search_time_ms", 40.0);
        near_end_min_progress_ = node_->declare_parameter<double>("search.near_end_min_progress", 0.4);
        tie_breaker_ = 1.0 + 1.0 / 10000;
    }

    bool KinodynamicAstar::posSafe(const Eigen::Vector2d &pos)
    {
        // 起点豁免: 已占据的区域不算未来碰撞（与安全监控 self_ignore 同一哲学）。
        // 贴墙/贴静态标注被 BRAKE 后，起点压障碍会让所有规划失败形成死锁
        if ((pos - start_pos_).norm() < start_ignore_radius_)
            return env_->isInMap(pos);
        // ESDF 验收替代旧 5 点二值 footprint 采样: 采样点间距 0.3m 会漏检
        // ≤0.2m 薄障碍带 (台沿/围栏)，规划器接受、监控 (ESDF hard) 必否 → 振荡
        return env_->isInMap(pos) && env_->getDistance(pos) >= accept_clearance_;
    }
    int KinodynamicAstar::search(Eigen::Vector2d start_pt, Eigen::Vector2d start_v, Eigen::Vector2d start_a,
                                 Eigen::Vector2d end_pt, Eigen::Vector2d end_v, bool init, bool dynamic, double time_start)
    {
        // 刷新地图原点 (滑动窗口可能已移动)
        env_->getMapRegion(origin_, map_size_3d_);

        start_vel_ = start_v;
        start_acc_ = start_a;
        start_pos_ = start_pt;

        // 起点贴障诊断: 豁免圈内起步合法但值得留痕 (曾静默失败 762 次无从归因)
        if (env_->isInMap(start_pt) && env_->getDistance(start_pt) < accept_clearance_)
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "A* start esdf %.2f < accept %.2f - exempt-radius departure",
                                 env_->getDistance(start_pt), accept_clearance_);

        PathNodePtr cur_node = path_node_pool_[0];
        cur_node->parent = NULL;
        cur_node->state.head(2) = start_pt;
        cur_node->state.tail(2) = start_v;
        cur_node->index = posToIndex(start_pt);
        cur_node->g_score = 0.0;

        Eigen::VectorXd end_state(4);
        Eigen::Vector2i end_index;
        double time_to_goal;
        end_state.head(2) = end_pt;
        end_state.tail(2) = end_v;
        end_index = posToIndex(end_pt);
        cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
        cur_node->node_state = IN_OPEN_SET;
        open_set_.push(cur_node);
        use_node_num_ += 1;

        if (dynamic)
        {
            time_origin_ = time_start;
            cur_node->time = time_start;
            cur_node->time_idx = timeToIndex(time_start);
            expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
            // cout << "time start: " << time_start << endl;
        }
        else
            expanded_nodes_.insert(cur_node->index, cur_node);

        bool init_search = init;
        const int tolerance = ceil(1 / resolution_);
        const auto search_t0 = std::chrono::steady_clock::now();
        // 打捞候选: 距目标最近的已扩展节点。搜索失败时退化返回部分路径
        // (NEAR_END)，机器人向目标推进后下一拍从更好的位置重来
        PathNodePtr best_node = NULL;
        double best_dist = std::numeric_limits<double>::max();
        bool pool_exhausted = false;
        const char *fail_reason = "open set empty";

        while (!open_set_.empty() && !pool_exhausted)
        {
            // 时间预算: 目标不可达时防止全域搜索拖死重规划回路
            if (max_search_time_ms_ > 0.0 &&
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - search_t0).count() > max_search_time_ms_)
            {
                fail_reason = "time budget";
                break;
            }

            cur_node = open_set_.top();
            open_set_.pop();
            cur_node->node_state = IN_CLOSE_SET;
            iter_num_ += 1;

            bool reach_horizon = (cur_node->state.head(2) - start_pt).norm() >= horizon_;
            bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance;

            if (near_end)
            {
                // near_end 是索引切比雪夫距离——"近"可能隔着薄障碍 (台沿死锁根因)。
                // shot 成功才算到达；失败不终止，继续扩展绕行，后续弹出的
                // near_end 节点 (如绕过障碍端头的) 再重试 shot
                estimateHeuristic(cur_node->state, end_state, time_to_goal);
                if (computeShotTraj(cur_node->state, end_state, time_to_goal))
                {
                    retrievePath(cur_node);
                    return REACH_END;
                }
            }
            if (reach_horizon)
            {
                retrievePath(cur_node);
                return REACH_HORIZON;
            }

            double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
            Eigen::Matrix<double, 4, 1> cur_state = cur_node->state;
            Eigen::Matrix<double, 4, 1> pro_state;
            vector<PathNodePtr> tmp_expand_nodes;
            Eigen::Vector2d um;
            double pro_t;
            vector<Eigen::Vector2d> inputs;
            vector<double> durations;
            if (init_search)
            {
                // Use all acceleration directions for init_search (not just start_acc_)
                // This is critical when start_vel=0 and start_acc=0, otherwise
                // the robot stays in the same voxel and all primitives are rejected.
                for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                    for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                    {
                        um << ax, ay;
                        inputs.push_back(um);
                    }
                for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
                     tau += time_res_init * init_max_tau_)
                    durations.push_back(tau);
                init_search = false;
            }
            else
            {
                for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                    for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                    {
                        um << ax, ay;
                        inputs.push_back(um);
                    }
                for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
                    durations.push_back(tau);
            }

            // cout << "cur state:" << cur_state.head(3).transpose() << endl;
            for (size_t i = 0; i < inputs.size() && !pool_exhausted; ++i)
                for (size_t j = 0; j < durations.size() && !pool_exhausted; ++j)
                {
                    um = inputs[i];
                    double tau = durations[j];
                    stateTransit(cur_state, pro_state, um, tau);
                    pro_t = cur_node->time + tau;
                    Eigen::Vector2d pro_pos = pro_state.head(2);

                    // Check if in close set
                    Eigen::Vector2i pro_id = posToIndex(pro_pos);
                    int pro_t_id = 0;
                    if (dynamic)
                        pro_t_id = timeToIndex(pro_t);

                    PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
                    if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
                    {
                        continue;
                    }

                    // Check maximal velocity
                    Eigen::Vector2d pro_v = pro_state.tail(2);
                    if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_)
                    {
                        continue;
                    }

                    // Check not in the same voxel
                    Eigen::Vector2i diff = pro_id - cur_node->index;
                    int diff_time = pro_t_id - cur_node->time_idx;
                    if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
                    {
                        continue;
                    }

                    // Check safety (豁免与 ESDF 验收统一在 posSafe)
                    Eigen::Vector2d pos;
                    Eigen::Matrix<double, 4, 1> xt;
                    bool is_occ = false;
                    for (int k = 1; k <= check_num_; ++k)
                    {
                        double dt = tau * double(k) / double(check_num_);
                        stateTransit(cur_state, xt, um, dt);
                        pos = xt.head(2);
                        if (!posSafe(pos))
                        {
                            is_occ = true;
                            break;
                        }
                    }
                    if (is_occ)
                    {
                        continue;
                    }

                    double time_to_goal, tmp_g_score, tmp_f_score;
                    tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
                    // 靠近障碍软惩罚 (rose A* clearance_weight 思想):
                    // 间隙不足 clearance_dist 的节点按不足量加代价，让搜索
                    // 在有余地时主动选宽敞走廊，而不是贴着可行边界走最短路
                    if (w_clearance_ > 0.0)
                    {
                        double esdf = env_->getDistance(Eigen::Vector2d(pro_state.head(2)));
                        double shortfall = clearance_dist_ - esdf;
                        if (shortfall > 0.0)
                            tmp_g_score += w_clearance_ * shortfall * tau;
                    }
                    tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

                    // Compare nodes expanded from the same parent
                    bool prune = false;
                    for (int j = 0; j < tmp_expand_nodes.size(); ++j)
                    {
                        PathNodePtr expand_node = tmp_expand_nodes[j];
                        if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
                        {
                            prune = true;
                            if (tmp_f_score < expand_node->f_score)
                            {
                                expand_node->f_score = tmp_f_score;
                                expand_node->g_score = tmp_g_score;
                                expand_node->state = pro_state;
                                expand_node->input = um;
                                expand_node->duration = tau;
                                if (dynamic)
                                    expand_node->time = cur_node->time + tau;
                            }
                            break;
                        }
                    }

                    // This node end up in a voxel different from others
                    if (!prune)
                    {
                        if (pro_node == NULL)
                        {
                            pro_node = path_node_pool_[use_node_num_];
                            pro_node->index = pro_id;
                            pro_node->state = pro_state;
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->input = um;
                            pro_node->duration = tau;
                            pro_node->parent = cur_node;
                            pro_node->node_state = IN_OPEN_SET;
                            if (dynamic)
                            {
                                pro_node->time = cur_node->time + tau;
                                pro_node->time_idx = timeToIndex(pro_node->time);
                            }
                            open_set_.push(pro_node);

                            // 打捞候选在创建时更新: 预算在首次扩展即耗尽时
                            // (init 一次生成数百 primitive)，弹出时更新会一无所获。
                            // 先过最小推进量再取距目标最近——否则朝目标的微小步
                            // (贴障时唯一能靠近的) 总是胜出又过不了推进门槛
                            double d_goal = (pro_state.head(2) - end_state.head(2)).norm();
                            if (d_goal < best_dist &&
                                (pro_state.head(2) - start_pt).norm() >= near_end_min_progress_)
                            {
                                best_dist = d_goal;
                                best_node = pro_node;
                            }

                            if (dynamic)
                                expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
                            else
                                expanded_nodes_.insert(pro_id, pro_node);

                            tmp_expand_nodes.push_back(pro_node);

                            use_node_num_ += 1;
                            if (use_node_num_ == allocate_num_)
                            {
                                // 不直接 return: 走统一收尾，打捞 best 节点部分路径
                                pool_exhausted = true;
                                fail_reason = "node pool exhausted";
                                break;
                            }
                        }
                        else if (pro_node->node_state == IN_OPEN_SET)
                        {
                            if (tmp_g_score < pro_node->g_score)
                            {
                                // pro_node->index = pro_id;
                                pro_node->state = pro_state;
                                pro_node->f_score = tmp_f_score;
                                pro_node->g_score = tmp_g_score;
                                pro_node->input = um;
                                pro_node->duration = tau;
                                pro_node->parent = cur_node;
                                if (dynamic)
                                    pro_node->time = cur_node->time + tau;
                            }
                        }
                        else
                        {
                            RCLCPP_ERROR(node_->get_logger(), "Error type in A* searching: %c", pro_node->node_state);
                        }
                    }
                }
            // init_search = false;
        }

        // 统一收尾: 到不了目标但有足够推进量 → 返回部分路径 (NEAR_END)，
        // 机器人先向目标挪，下一拍从更好的位置重规划；否则有声 NO_PATH——
        // 静默失败出口曾让 762 次规划失败无从归因 (台沿死锁取证)
        if (best_node != NULL)
        {
            retrievePath(best_node);
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "A* partial path (%s): best node %.2fm from goal, nodes=%d iters=%d",
                                 fail_reason, best_dist, use_node_num_, iter_num_);
            return NEAR_END;
        }
        double elapsed_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - search_t0).count();
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                             "A* no path (%s): start(%.2f,%.2f) esdf %.2f -> goal(%.2f,%.2f) esdf %.2f, "
                             "nodes=%d iters=%d %.1fms",
                             fail_reason, start_pt(0), start_pt(1),
                             env_->isInMap(start_pt) ? env_->getDistance(start_pt) : -1.0,
                             end_pt(0), end_pt(1),
                             env_->isInMap(end_pt) ? env_->getDistance(end_pt) : -1.0,
                             use_node_num_, iter_num_, elapsed_ms);
        return NO_PATH;
    }

    double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
    {
        const Eigen::Vector2d dp = x2.head(2) - x1.head(2);
        const Eigen::Vector2d v0 = x1.segment(2, 2);
        const Eigen::Vector2d v1 = x2.segment(2, 2);

        double c1 = -36 * dp.dot(dp);
        double c2 = 24 * (v0 + v1).dot(dp);
        double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
        double c4 = 0;
        double c5 = w_time_;

        std::vector<double> ts = solveQuartic(c5, c4, c3, c2, c1);

        double v_max = max_vel_ * 0.5;
        double t_bar = (x1.head(2) - x2.head(2)).lpNorm<Eigen::Infinity>() / v_max;
        ts.push_back(t_bar);

        double cost = 100000000;
        double t_d = t_bar;

        for (auto t : ts)
        {
            if (t < t_bar)
                continue;
            double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
            if (c < cost)
            {
                cost = c;
                t_d = t;
            }
        }

        optimal_time = t_d;
        return 1.0 * (1 + tie_breaker_) * cost;
    }
    bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
    {
        /* ---------- get coefficient ---------- */
        const Eigen::Vector2d p0 = state1.head(2);
        const Eigen::Vector2d dp = state2.head(2) - p0;
        const Eigen::Vector2d v0 = state1.segment(2, 2);
        const Eigen::Vector2d v1 = state2.segment(2, 2);
        const Eigen::Vector2d dv = v1 - v0;
        double t_d = time_to_goal;
        Eigen::MatrixXd coef(2, 4);
        end_vel_ = v1;

        Eigen::Vector2d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
        Eigen::Vector2d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
        Eigen::Vector2d c = v0;
        Eigen::Vector2d d = p0;

        // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
        // a*t^3 + b*t^2 + v0*t + p0
        coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

        Eigen::Vector2d coord, vel, acc;
        Eigen::VectorXd poly1d, t, polyv, polya;
        Eigen::Vector2i index;

        Eigen::MatrixXd Tm(4, 4);
        Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

        /* ---------- forward checking of trajectory ---------- */
        double t_delta = t_d / 10;
        for (double time = t_delta; time <= t_d; time += t_delta)
        {
            t = Eigen::VectorXd::Zero(4);
            for (int j = 0; j < 4; j++)
                t(j) = pow(time, j);

            for (int dim = 0; dim < 2; dim++)
            {
                poly1d = coef.row(dim);
                coord(dim) = poly1d.dot(t);
                vel(dim) = (Tm * poly1d).dot(t);
                acc(dim) = (Tm * Tm * poly1d).dot(t);

                if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
                {
                    // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
                    // return false;
                }
            }

            if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1))
            {
                return false;
            }

            // ESDF 验收 + 起点豁免 (与扩展检查同一判据)。豁免必不可少:
            // 贴墙起步的近距 shot 前几个采样必然低于验收阈值
            if (!posSafe(coord))
                return false;
        }
        coef_shot_ = coef;
        t_shot_ = t_d;
        is_shot_succ_ = true;
        return true;
    }
    void KinodynamicAstar::retrievePath(PathNodePtr end_node)
    {
        PathNodePtr cur_node = end_node;
        path_nodes_.push_back(cur_node);

        while (cur_node->parent != NULL)
        {
            cur_node = cur_node->parent;
            path_nodes_.push_back(cur_node);
        }

        reverse(path_nodes_.begin(), path_nodes_.end());
    }
    std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
    {
        vector<PathNodePtr> visited;
        visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
        return visited;
    }
    int KinodynamicAstar::timeToIndex(double time)
    {
        int idx = floor((time - time_origin_) * inv_time_resolution_);
        return idx;
    }
    Eigen::Vector2i KinodynamicAstar::posToIndex(Eigen::Vector2d pt)
    {
        Eigen::Vector2i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

        // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
        // origin_(1)) * inv_resolution_),
        //     floor((pt(2) - origin_(2)) * inv_resolution_);

        return idx;
    }
    void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 4, 1> &state0, Eigen::Matrix<double, 4, 1> &state1,
                                        Eigen::Vector2d um, double tau)
    {
        for (int i = 0; i < 2; ++i)
            phi_(i, i + 2) = tau;

        Eigen::Matrix<double, 4, 1> integral;
        integral.head(2) = 0.5 * pow(tau, 2) * um;
        integral.tail(2) = tau * um;

        state1 = phi_ * state0 + integral;
    }
    void KinodynamicAstar::setEnvironment(sentry_nav::EnvironmentInterface *env)
    {
        this->env_ = env;
    }
    void KinodynamicAstar::reset()
    {
        expanded_nodes_.clear();
        path_nodes_.clear();

        std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
        open_set_.swap(empty_queue);

        for (int i = 0; i < use_node_num_; i++)
        {
            PathNodePtr node = path_node_pool_[i];
            node->parent = NULL;
            node->node_state = NOT_EXPAND;
        }

        use_node_num_ = 0;
        iter_num_ = 0;
        is_shot_succ_ = false;
        has_path_ = false;
    }

    std::vector<Eigen::Vector2d> KinodynamicAstar::getKinoTraj(double delta_t)
    {
        vector<Eigen::Vector2d> state_list;
        if (path_nodes_.empty())
            return state_list;  // NO_PATH 后调用的防护
        RCLCPP_DEBUG(node_->get_logger(), "getKinoTraj: path_nodes=%zu", path_nodes_.size());
        /* ---------- get traj of searching ---------- */
        PathNodePtr node = path_nodes_.back();
        Eigen::Matrix<double, 4, 1> x0, xt;

        while (node->parent != NULL)
        {
            Eigen::Vector2d ut = node->input;
            double duration = node->duration;
            x0 = node->parent->state;

            for (double t = duration; t >= -1e-5; t -= delta_t)
            {
                stateTransit(x0, xt, ut, t);
                state_list.push_back(xt.head(2));
            }
            node = node->parent;
        }
        reverse(state_list.begin(), state_list.end());
        // /* ---------- get traj of one shot ---------- */
        if (is_shot_succ_)
        {
            Eigen::Vector2d coord;
            Eigen::VectorXd poly1d, time(4);

            for (double t = delta_t; t <= t_shot_; t += delta_t)
            {
                for (int j = 0; j < 4; j++)
                    time(j) = pow(t, j);

                for (int dim = 0; dim < 2; dim++)
                {
                    poly1d = coef_shot_.row(dim);
                    coord(dim) = poly1d.dot(time);
                }
                state_list.push_back(coord);
            }
        }

        return state_list;
    }
    void KinodynamicAstar::getSamples(double &ts, vector<Eigen::Vector2d> &point_set,
                                      vector<Eigen::Vector2d> &start_end_derivatives)
    {
        if (path_nodes_.empty())
            return;  // NO_PATH 后调用的防护
        /* ---------- path duration ---------- */
        double T_sum = 0.0;
        if (is_shot_succ_)
            T_sum += t_shot_;
        PathNodePtr node = path_nodes_.back();
        while (node->parent != NULL)
        {
            T_sum += node->duration;
            node = node->parent;
        }
        // cout << "duration:" << T_sum << endl;

        // Calculate boundary vel and acc
        Eigen::Vector2d end_vel, end_acc;
        double t;
        if (is_shot_succ_)
        {
            t = t_shot_;
            end_vel = end_vel_;
            for (int dim = 0; dim < 2; ++dim)
            {
                Eigen::Vector4d coe = coef_shot_.row(dim);
                end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
            }
        }
        else
        {
            t = path_nodes_.back()->duration;
            // 末速取路径末节点 (node 此刻已沿 parent 走到根 = 起点——
            // 旧代码把起点速度当末速喂给 MINCO，NEAR_END 常态化后必须修)
            end_vel = path_nodes_.back()->state.tail(2);
            end_acc = path_nodes_.back()->input;
        }

        // Get point samples
        int seg_num = floor(T_sum / ts);
        seg_num = max(8, seg_num);
        ts = T_sum / double(seg_num);
        bool sample_shot_traj = is_shot_succ_;
        node = path_nodes_.back();

        for (double ti = T_sum; ti > -1e-5; ti -= ts)
        {
            if (sample_shot_traj)
            {
                // samples on shot traj
                Eigen::Vector2d coord;
                Eigen::Vector4d poly1d, time;

                for (int j = 0; j < 4; j++)
                    time(j) = pow(t, j);

                for (int dim = 0; dim < 2; dim++)
                {
                    poly1d = coef_shot_.row(dim);
                    coord(dim) = poly1d.dot(time);
                }

                point_set.push_back(coord);
                t -= ts;

                /* end of segment */
                if (t < -1e-5)
                {
                    sample_shot_traj = false;
                    if (node->parent != NULL)
                        t += node->duration;
                }
            }
            else
            {
                // samples on searched traj
                Eigen::Matrix<double, 4, 1> x0 = node->parent->state;
                Eigen::Matrix<double, 4, 1> xt;
                Eigen::Vector2d ut = node->input;

                stateTransit(x0, xt, ut, t);

                point_set.push_back(xt.head(2));
                t -= ts;

                // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
                if (t < -1e-5 && node->parent->parent != NULL)
                {
                    node = node->parent;
                    t += node->duration;
                    if (t < 0) t = 0;  // clamp to prevent backward stateTransit
                }
            }
        }
        reverse(point_set.begin(), point_set.end());

        // calculate start acc
        Eigen::Vector2d start_acc;
        if (path_nodes_.back()->parent == NULL)
        {
            // no searched traj, calculate by shot traj
            start_acc = 2 * coef_shot_.col(2);
        }
        else
        {
            // input of searched traj
            start_acc = node->input;
        }

        start_end_derivatives.push_back(start_vel_);
        start_end_derivatives.push_back(end_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(end_acc);
    }
}