#include "sentry_trav_rviz_plugin/traversability_panel.hpp"

#include <cmath>
#include <fstream>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include "rviz_common/display_context.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sentry_trav_rviz_plugin/annotation_bridge.hpp"

namespace sentry_trav_rviz_plugin
{

namespace
{
// 按类型给预览上色 (与 plan_env 语义对应): free 绿 / obstacle 红 / oneway 蓝。
void setTypeColor(std_msgs::msg::ColorRGBA & c, int type)
{
    c.a = 1.0f;
    switch (type) {
        case 1:  c.r = 0.90f; c.g = 0.10f; c.b = 0.10f; break;  // obstacle
        case 2:  c.r = 0.20f; c.g = 0.55f; c.b = 1.00f; break;  // oneway
        default: c.r = 0.20f; c.g = 0.85f; c.b = 0.25f; break;  // free
    }
}
constexpr double kMarkerZ = 0.10;  // 抬高一点避免与地面网格 z-fighting
}  // namespace

TraversabilityPanel::TraversabilityPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
    // ---- 标注属性 ----
    type_combo_ = new QComboBox;
    type_combo_->addItem("Free");      // 0
    type_combo_->addItem("Obstacle");  // 1
    type_combo_->addItem("Oneway");    // 2

    dir_spin_ = new QDoubleSpinBox;
    dir_spin_->setRange(-180.0, 180.0);
    dir_spin_->setDecimals(1);
    dir_spin_->setSuffix(" deg");
    dir_spin_->setValue(0.0);

    tol_spin_ = new QDoubleSpinBox;
    tol_spin_->setRange(0.0, 180.0);
    tol_spin_->setDecimals(1);
    tol_spin_->setSuffix(" deg");
    tol_spin_->setValue(90.0);  // 默认前向半球

    id_edit_ = new QLineEdit;
    id_edit_->setPlaceholderText("region id (留空自动命名)");

    auto * attr_form = new QFormLayout;
    attr_form->addRow("Type", type_combo_);
    attr_form->addRow("Direction", dir_spin_);
    attr_form->addRow("Tolerance", tol_spin_);
    attr_form->addRow("Id", id_edit_);
    auto * attr_box = new QGroupBox("Region attributes");
    attr_box->setLayout(attr_form);

    // direction 仅对 oneway 有意义, 非 oneway 时灰掉
    auto sync_dir_enabled = [this](int idx) {
        const bool oneway = (idx == 2);
        dir_spin_->setEnabled(oneway);
    };
    connect(type_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, sync_dir_enabled);
    sync_dir_enabled(type_combo_->currentIndex());

    // ---- 绘制按钮 ----
    new_btn_ = new QPushButton("New Region");
    finish_btn_ = new QPushButton("Finish Region");
    finish_btn_->setEnabled(false);
    auto * draw_row = new QHBoxLayout;
    draw_row->addWidget(new_btn_);
    draw_row->addWidget(finish_btn_);
    connect(new_btn_, &QPushButton::clicked, this, &TraversabilityPanel::onNewRegion);
    connect(finish_btn_, &QPushButton::clicked, this, &TraversabilityPanel::onFinishRegion);

    // 方向拾取: 在地图点 起点→终点, 自动算 direction_deg (仅 oneway 有意义)
    pick_dir_btn_ = new QPushButton("Pick Direction (2 pts)");
    connect(pick_dir_btn_, &QPushButton::clicked, this, &TraversabilityPanel::onPickDirection);

    // ---- 区域列表 ----
    region_list_ = new QListWidget;
    // 选中已画区域 → 把它的属性载回上方表单 (配合 Apply 修改已画区域)
    connect(region_list_, &QListWidget::currentRowChanged,
            this, &TraversabilityPanel::onSelectRegion);
    apply_btn_ = new QPushButton("Apply to Selected");
    connect(apply_btn_, &QPushButton::clicked, this, &TraversabilityPanel::onApplyRegion);
    auto * del_btn = new QPushButton("Delete Selected");
    connect(del_btn, &QPushButton::clicked, this, &TraversabilityPanel::onDeleteRegion);

    // ---- 地图元信息 (写入 .trav.yaml 的 map 段) ----
    res_spin_ = new QDoubleSpinBox;
    res_spin_->setRange(0.005, 1.0);
    res_spin_->setDecimals(3);
    res_spin_->setSingleStep(0.005);
    res_spin_->setValue(0.05);
    origin_x_spin_ = new QDoubleSpinBox;
    origin_x_spin_->setRange(-10000.0, 10000.0);
    origin_x_spin_->setDecimals(3);
    origin_x_spin_->setValue(-3.58);
    origin_y_spin_ = new QDoubleSpinBox;
    origin_y_spin_->setRange(-10000.0, 10000.0);
    origin_y_spin_->setDecimals(3);
    origin_y_spin_->setValue(-9.44);
    default_tol_spin_ = new QDoubleSpinBox;
    default_tol_spin_->setRange(0.0, 180.0);
    default_tol_spin_->setDecimals(1);
    default_tol_spin_->setValue(90.0);

    auto * map_form = new QFormLayout;
    map_form->addRow("Resolution", res_spin_);
    map_form->addRow("Origin x", origin_x_spin_);
    map_form->addRow("Origin y", origin_y_spin_);
    map_form->addRow("Default tol", default_tol_spin_);
    auto * map_box = new QGroupBox("Map meta (width/height 自动 bbox)");
    map_box->setLayout(map_form);

    // ---- 读写 ----
    auto * load_btn = new QPushButton("Load .trav.yaml");
    auto * save_btn = new QPushButton("Save .trav.yaml");
    auto * io_row = new QHBoxLayout;
    io_row->addWidget(load_btn);
    io_row->addWidget(save_btn);
    connect(load_btn, &QPushButton::clicked, this, &TraversabilityPanel::onLoad);
    connect(save_btn, &QPushButton::clicked, this, &TraversabilityPanel::onSave);

    // ---- 总布局 ----
    auto * root = new QVBoxLayout;
    root->addWidget(attr_box);
    root->addLayout(draw_row);
    root->addWidget(pick_dir_btn_);
    root->addWidget(new QLabel("Regions:"));
    root->addWidget(region_list_);
    auto * edit_row = new QHBoxLayout;
    edit_row->addWidget(apply_btn_);
    edit_row->addWidget(del_btn);
    root->addLayout(edit_row);
    root->addWidget(map_box);
    root->addLayout(io_row);
    setLayout(root);
}

TraversabilityPanel::~TraversabilityPanel() = default;

void TraversabilityPanel::onInitialize()
{
    // 拿到 RViz 共享的 rclcpp 节点, 建预览发布器 + clicked_point 兜底订阅。
    auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (ros_node_abstraction) {
        node_ = ros_node_abstraction->get_raw_node();
        // transient_local: 后加入的 MarkerArray Display 也能立刻看到最近一帧标注
        preview_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/traversability_annotation_preview", rclcpp::QoS(1).transient_local());

        // 便捷通道: 内置 "Publish Point" 工具发布的点, 绘制中时也并入当前多边形
        clicked_sub_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", rclcpp::QoS(10),
            [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
                handlePoint(msg->point.x, msg->point.y);
            });
    }

    // 接 Tool ← → Panel 的信号桥
    auto * bridge = AnnotationBridge::instance();
    connect(bridge, &AnnotationBridge::pointAdded, this, &TraversabilityPanel::onToolPoint);
    connect(bridge, &AnnotationBridge::regionFinished, this, &TraversabilityPanel::onToolFinish);
    connect(bridge, &AnnotationBridge::pointUndone, this, &TraversabilityPanel::onToolUndo);

    publishPreview();
}

// ---------- 绘制流程 ----------

void TraversabilityPanel::onNewRegion()
{
    // 把当前 UI 属性快照进 current_, 开始收点。
    current_ = Region{};
    current_.id = id_edit_->text().toStdString();
    current_.type = type_combo_->currentIndex();
    current_.dir_deg = dir_spin_->value();
    current_.tol_deg = tol_spin_->value();
    current_.polygon.clear();
    drawing_ = true;

    finish_btn_->setEnabled(true);
    new_btn_->setEnabled(false);
    if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "[trav] 开始绘制 region (左键取点, 右键/Esc 结束)");
    }
    publishPreview();
}

void TraversabilityPanel::onFinishRegion()
{
    if (picking_dir_) {   // 方向拾取中: 右键/Esc/Finish 视为取消拾取
        picking_dir_ = false;
        dir_pts_.clear();
        if (node_) {
            RCLCPP_INFO(node_->get_logger(), "[trav] 方向拾取已取消");
        }
        return;
    }
    if (drawing_) {
        if (current_.polygon.size() >= 3) {
            if (current_.id.empty()) {
                current_.id = "region_" + std::to_string(regions_.size());
            }
            regions_.push_back(current_);
        } else {
            QMessageBox::warning(this, "Traversability",
                                 "多边形至少需要 3 个顶点, 已丢弃本次绘制。");
        }
    }
    drawing_ = false;
    current_ = Region{};
    finish_btn_->setEnabled(false);
    new_btn_->setEnabled(true);
    refreshList();
    publishPreview();
}

void TraversabilityPanel::onDeleteRegion()
{
    const int row = region_list_->currentRow();
    if (row >= 0 && row < static_cast<int>(regions_.size())) {
        regions_.erase(regions_.begin() + row);
        refreshList();
        publishPreview();
    }
}

// 选中列表里某区域 → 把它的属性载回上方表单 (查看/准备修改)。绘制/拾取中不打断。
void TraversabilityPanel::onSelectRegion(int row)
{
    if (drawing_ || picking_dir_) {
        return;
    }
    if (row < 0 || row >= static_cast<int>(regions_.size())) {
        return;
    }
    const Region & r = regions_[row];
    type_combo_->setCurrentIndex(r.type);
    dir_spin_->setValue(r.dir_deg);
    tol_spin_->setValue(r.tol_deg);
    id_edit_->setText(QString::fromStdString(r.id));
}

// 把上方表单的属性写回选中区域 (改 类型/方向/容差/id; 多边形顶点不变)。
void TraversabilityPanel::onApplyRegion()
{
    const int row = region_list_->currentRow();
    if (row < 0 || row >= static_cast<int>(regions_.size())) {
        QMessageBox::information(this, "Traversability",
                                 "请先在 Regions 列表里选中一个区域。");
        return;
    }
    Region & r = regions_[row];
    r.type = type_combo_->currentIndex();
    r.dir_deg = dir_spin_->value();
    r.tol_deg = tol_spin_->value();
    const std::string new_id = id_edit_->text().toStdString();
    if (!new_id.empty()) {
        r.id = new_id;
    }
    refreshList();
    region_list_->setCurrentRow(row);  // refreshList 会清空选择, 这里恢复
    publishPreview();
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[trav] 已更新区域 #%d (%s)", row, r.id.c_str());
    }
}

// 进入方向拾取态: 在地图依次点 起点→终点, 自动 atan2 算 direction_deg。
void TraversabilityPanel::onPickDirection()
{
    if (drawing_) {
        QMessageBox::information(this, "Traversability",
                                 "请先 Finish 当前多边形, 再拾取方向。");
        return;
    }
    // 方向只对 oneway 有意义, 自动切到 oneway 让 Direction 框可用
    if (type_combo_->currentIndex() != 2) {
        type_combo_->setCurrentIndex(2);
    }
    picking_dir_ = true;
    dir_pts_.clear();
    if (node_) {
        RCLCPP_INFO(node_->get_logger(),
                    "[trav] 方向拾取: 在地图依次点 起点→终点 (沿允许行进方向); 右键取消");
    }
}

// 收点分流: 方向拾取态收两点算角度; 否则交给多边形绘制 (addPoint 内部判 drawing_)。
void TraversabilityPanel::handlePoint(double x, double y)
{
    if (picking_dir_) {
        dir_pts_.emplace_back(x, y);
        if (dir_pts_.size() >= 2) {
            const double dx = dir_pts_[1].first - dir_pts_[0].first;
            const double dy = dir_pts_[1].second - dir_pts_[0].second;
            if (std::hypot(dx, dy) > 1e-6) {
                const double deg = std::atan2(dy, dx) * 180.0 / M_PI;
                dir_spin_->setValue(deg);
                if (node_) {
                    RCLCPP_INFO(node_->get_logger(), "[trav] direction_deg = %.1f", deg);
                }
            } else if (node_) {
                RCLCPP_WARN(node_->get_logger(), "[trav] 两点重合, 方向无效, 请重试");
            }
            picking_dir_ = false;
            dir_pts_.clear();
        }
        return;
    }
    addPoint(x, y);
}

void TraversabilityPanel::onToolPoint(double x, double y)
{
    handlePoint(x, y);
}

void TraversabilityPanel::onToolFinish()
{
    onFinishRegion();
}

void TraversabilityPanel::onToolUndo()
{
    if (drawing_ && !current_.polygon.empty()) {
        current_.polygon.pop_back();
        publishPreview();
    }
}

void TraversabilityPanel::addPoint(double x, double y)
{
    if (!drawing_) {
        return;  // 未处于绘制态, 忽略 (避免误收 clicked_point)
    }
    current_.polygon.emplace_back(x, y);
    publishPreview();
}

// ---------- 列表 / 预览 ----------

void TraversabilityPanel::refreshList()
{
    region_list_->clear();
    for (const auto & r : regions_) {
        QString line = QString::fromStdString(r.id) + "  [" + typeName(r.type) + "]";
        if (r.type == 2) {
            line += QString("  dir=%1 tol=%2").arg(r.dir_deg).arg(r.tol_deg);
        }
        line += QString("  (%1 pts)").arg(r.polygon.size());
        region_list_->addItem(line);
    }
}

std::string TraversabilityPanel::activeFrame() const
{
    if (auto * ctx = getDisplayContext()) {
        const QString f = ctx->getFixedFrame();
        if (!f.isEmpty()) {
            return f.toStdString();
        }
    }
    return "map";
}

void TraversabilityPanel::publishPreview()
{
    if (!preview_pub_ || !node_) {
        return;
    }

    const std::string frame = activeFrame();
    const auto stamp = node_->now();

    visualization_msgs::msg::MarkerArray arr;

    // 先发一个 DELETEALL 清掉上一帧, 避免删除区域后残留。
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = frame;
        del.header.stamp = stamp;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
    }

    // 统一构造一个基底 Marker
    auto make_base = [&](const std::string & ns, int id, int32_t mtype) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = frame;
        m.header.stamp = stamp;
        m.ns = ns;
        m.id = id;
        m.type = mtype;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;  // 单位四元数, 避免 RViz 警告
        return m;
    };

    // 闭合多边形 LINE_STRIP
    auto add_polygon = [&](const std::string & ns, int id,
                           const std::vector<std::pair<double, double>> & poly,
                           const std_msgs::msg::ColorRGBA & color) {
        if (poly.size() < 2) {
            return;
        }
        auto m = make_base(ns, id, visualization_msgs::msg::Marker::LINE_STRIP);
        m.scale.x = 0.05;  // 线宽
        m.color = color;
        for (const auto & p : poly) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = kMarkerZ;
            m.points.push_back(pt);
        }
        // 回到首点闭合
        geometry_msgs::msg::Point first;
        first.x = poly.front().first;
        first.y = poly.front().second;
        first.z = kMarkerZ;
        m.points.push_back(first);
        arr.markers.push_back(m);
    };

    auto centroid = [](const std::vector<std::pair<double, double>> & poly) {
        double cx = 0.0, cy = 0.0;
        for (const auto & p : poly) {
            cx += p.first;
            cy += p.second;
        }
        const double n = static_cast<double>(poly.size());
        return std::make_pair(cx / n, cy / n);
    };

    int id = 0;
    for (const auto & r : regions_) {
        std_msgs::msg::ColorRGBA color;
        setTypeColor(color, r.type);
        add_polygon("trav_poly", id, r.polygon, color);

        if (r.type == 2 && !r.polygon.empty()) {
            // oneway 方向箭头: 质心指向 direction_deg
            auto c = centroid(r.polygon);
            const double a = r.dir_deg * M_PI / 180.0;
            const double len = 0.8;
            auto m = make_base("trav_dir", id, visualization_msgs::msg::Marker::ARROW);
            m.scale.x = 0.06;  // 杆径
            m.scale.y = 0.14;  // 箭头直径
            m.scale.z = 0.20;  // 箭头长度
            m.color = color;
            geometry_msgs::msg::Point s, e;
            s.x = c.first;             s.y = c.second;             s.z = kMarkerZ;
            e.x = c.first + len * std::cos(a);
            e.y = c.second + len * std::sin(a);
            e.z = kMarkerZ;
            m.points.push_back(s);
            m.points.push_back(e);
            arr.markers.push_back(m);
        }

        if (!r.polygon.empty()) {
            // 文字标签
            auto c = centroid(r.polygon);
            auto m = make_base("trav_text", id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            m.scale.z = 0.30;
            m.color.r = m.color.g = m.color.b = 1.0f;
            m.color.a = 1.0f;
            m.pose.position.x = c.first;
            m.pose.position.y = c.second;
            m.pose.position.z = kMarkerZ + 0.25;
            m.text = r.id + " [" + typeName(r.type) + "]";
            arr.markers.push_back(m);
        }
        ++id;
    }

    // 进行中的多边形 (黄色, 顶点高亮)
    if (drawing_ && !current_.polygon.empty()) {
        std_msgs::msg::ColorRGBA yellow;
        yellow.r = 1.0f; yellow.g = 0.9f; yellow.b = 0.1f; yellow.a = 1.0f;
        add_polygon("trav_wip", 0, current_.polygon, yellow);

        auto m = make_base("trav_wip_pts", 0, visualization_msgs::msg::Marker::SPHERE_LIST);
        m.scale.x = m.scale.y = m.scale.z = 0.12;
        m.color = yellow;
        for (const auto & p : current_.polygon) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = kMarkerZ;
            m.points.push_back(pt);
        }
        arr.markers.push_back(m);
    }

    preview_pub_->publish(arr);
}

// ---------- 读写 .trav.yaml ----------

const char * TraversabilityPanel::typeName(int t)
{
    switch (t) {
        case 1:  return "obstacle";
        case 2:  return "oneway";
        default: return "free";
    }
}

int TraversabilityPanel::parseType(const std::string & s)
{
    if (s == "obstacle" || s == "OBSTACLE") {
        return 1;
    }
    if (s == "oneway" || s == "ONEWAY" || s == "one_way") {
        return 2;
    }
    return 0;
}

void TraversabilityPanel::onSave()
{
    const QString fn = QFileDialog::getSaveFileName(
        this, "Save .trav.yaml", QString(), "Traversability YAML (*.trav.yaml *.yaml)");
    if (fn.isEmpty()) {
        return;
    }

    // 用 YAML::Node 构造, 控制 origin / polygon 为 flow 风格, 贴合 plan_env 约定格式。
    YAML::Node root;

    YAML::Node map;
    map["resolution"] = res_spin_->value();
    YAML::Node origin;
    origin.push_back(origin_x_spin_->value());
    origin.push_back(origin_y_spin_->value());
    origin.SetStyle(YAML::EmitterStyle::Flow);
    map["origin"] = origin;
    map["width"] = 0;   // 0 => 由 regions 包围盒自动推导
    map["height"] = 0;
    root["map"] = map;

    root["default_tolerance_deg"] = default_tol_spin_->value();

    YAML::Node regions(YAML::NodeType::Sequence);
    for (const auto & r : regions_) {
        YAML::Node rn;
        rn["id"] = r.id;
        rn["type"] = std::string(typeName(r.type));
        if (r.type == 2) {  // direction/tolerance 仅 oneway 有意义
            rn["direction_deg"] = r.dir_deg;
            rn["tolerance_deg"] = r.tol_deg;
        }
        YAML::Node poly(YAML::NodeType::Sequence);
        for (const auto & p : r.polygon) {
            YAML::Node v;
            v.push_back(p.first);
            v.push_back(p.second);
            v.SetStyle(YAML::EmitterStyle::Flow);
            poly.push_back(v);
        }
        rn["polygon"] = poly;
        regions.push_back(rn);
    }
    root["regions"] = regions;

    std::ofstream fout(fn.toStdString());
    if (!fout) {
        QMessageBox::critical(this, "Traversability",
                              "无法写入文件: " + fn);
        return;
    }
    YAML::Emitter emitter;
    emitter << root;
    fout << "# generated by sentry_trav_rviz_plugin\n";
    fout << emitter.c_str() << "\n";
    fout.close();

    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[trav] 已保存 %zu 个区域到 %s",
                    regions_.size(), fn.toStdString().c_str());
    }
}

void TraversabilityPanel::onLoad()
{
    const QString fn = QFileDialog::getOpenFileName(
        this, "Load .trav.yaml", QString(), "Traversability YAML (*.trav.yaml *.yaml)");
    if (fn.isEmpty()) {
        return;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(fn.toStdString());
    } catch (const std::exception & e) {
        QMessageBox::critical(this, "Traversability",
                              QString("解析失败: ") + e.what());
        return;
    }

    std::vector<Region> loaded;
    try {
        if (root["map"]) {
            const auto & m = root["map"];
            if (m["resolution"]) {
                res_spin_->setValue(m["resolution"].as<double>());
            }
            if (m["origin"] && m["origin"].size() >= 2) {
                origin_x_spin_->setValue(m["origin"][0].as<double>());
                origin_y_spin_->setValue(m["origin"][1].as<double>());
            }
        }
        if (root["default_tolerance_deg"]) {
            default_tol_spin_->setValue(root["default_tolerance_deg"].as<double>());
        }
        const double default_tol = default_tol_spin_->value();

        if (root["regions"]) {
            for (const auto & rn : root["regions"]) {
                Region reg;
                reg.id = rn["id"] ? rn["id"].as<std::string>() : "";
                reg.type = parseType(rn["type"] ? rn["type"].as<std::string>() : "free");
                reg.dir_deg = rn["direction_deg"] ? rn["direction_deg"].as<double>() : 0.0;
                reg.tol_deg = rn["tolerance_deg"] ? rn["tolerance_deg"].as<double>() : default_tol;
                if (rn["polygon"]) {
                    for (const auto & v : rn["polygon"]) {
                        if (v.size() >= 2) {
                            reg.polygon.emplace_back(v[0].as<double>(), v[1].as<double>());
                        }
                    }
                }
                if (reg.polygon.size() >= 3) {
                    loaded.push_back(std::move(reg));
                }
            }
        }
    } catch (const std::exception & e) {
        QMessageBox::critical(this, "Traversability",
                              QString("字段解析错误: ") + e.what());
        return;
    }

    regions_ = std::move(loaded);
    drawing_ = false;
    current_ = Region{};
    finish_btn_->setEnabled(false);
    new_btn_->setEnabled(true);
    refreshList();
    publishPreview();

    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "[trav] 已从 %s 载入 %zu 个区域",
                    fn.toStdString().c_str(), regions_.size());
    }
}

}  // namespace sentry_trav_rviz_plugin

PLUGINLIB_EXPORT_CLASS(sentry_trav_rviz_plugin::TraversabilityPanel, rviz_common::Panel)
