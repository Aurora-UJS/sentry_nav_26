#pragma once
/**
 * TraversabilityPanel: rviz_common::Panel 子类 (Qt5 Widget)。
 *
 * 职责:
 *   - 选择标注类型 (Free/Obstacle/Oneway)、oneway 的 direction_deg / tolerance_deg、区域 id;
 *   - "New Region" 开始绘制、"Finish Region" 结束 (与 TraversabilityTool 协作收点);
 *   - 维护已标注区域列表, 支持删除;
 *   - "Load" / "Save" 读写 plan_env 约定的 .trav.yaml (yaml-cpp);
 *   - 在 /traversability_annotation_preview 上发布 visualization_msgs/MarkerArray
 *     预览 (多边形 LINE_STRIP 按类型着色 + oneway 方向箭头)。
 *
 * 收点来源 (二选一或并用):
 *   1) TraversabilityTool 经 AnnotationBridge 转发 (推荐, 真正的 RViz Tool);
 *   2) 内置 "Publish Point" 工具发布的 /clicked_point (作为兜底/便捷通道)。
 */

#include <string>
#include <utility>
#include <vector>

#include "rviz_common/panel.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QListWidget;
class QPushButton;

namespace sentry_trav_rviz_plugin
{

class TraversabilityPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit TraversabilityPanel(QWidget * parent = nullptr);
    ~TraversabilityPanel() override;

    void onInitialize() override;

private Q_SLOTS:
    void onNewRegion();      // 开始一段新多边形
    void onFinishRegion();   // 结束并提交当前多边形
    void onDeleteRegion();   // 删除列表选中区域
    void onLoad();           // 读 .trav.yaml
    void onSave();           // 写 .trav.yaml

    // --- 来自 AnnotationBridge 的工具事件 ---
    void onToolPoint(double x, double y);
    void onToolFinish();
    void onToolUndo();

private:
    // 类型枚举与 plan_env 一致: 0 free / 1 obstacle / 2 oneway
    struct Region
    {
        std::string id;
        int type = 0;
        double dir_deg = 0.0;
        double tol_deg = 90.0;
        std::vector<std::pair<double, double>> polygon;  // 世界坐标 (米)
    };

    void addPoint(double x, double y);   // 收点统一入口 (tool / clicked_point 共用)
    void refreshList();                  // 同步区域列表 UI
    void publishPreview();               // 重新发布 MarkerArray 预览
    std::string activeFrame() const;     // 当前 RViz fixed frame (默认 map)
    static const char * typeName(int t);
    static int parseType(const std::string & s);

    // --- UI ---
    QComboBox * type_combo_ = nullptr;
    QDoubleSpinBox * dir_spin_ = nullptr;
    QDoubleSpinBox * tol_spin_ = nullptr;
    QLineEdit * id_edit_ = nullptr;
    QDoubleSpinBox * res_spin_ = nullptr;
    QDoubleSpinBox * origin_x_spin_ = nullptr;
    QDoubleSpinBox * origin_y_spin_ = nullptr;
    QDoubleSpinBox * default_tol_spin_ = nullptr;
    QListWidget * region_list_ = nullptr;
    QPushButton * new_btn_ = nullptr;
    QPushButton * finish_btn_ = nullptr;

    // --- 数据 ---
    std::vector<Region> regions_;
    Region current_;        // 进行中的多边形 (含本段元信息快照)
    bool drawing_ = false;

    // --- ROS ---
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr preview_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
};

}  // namespace sentry_trav_rviz_plugin
