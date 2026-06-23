#pragma once
/**
 * TraversabilityTool: rviz_common::Tool 子类。
 *
 *   左键   → 把屏幕点投影到地面 XY 平面 (z=0), 经 AnnotationBridge 把 (x,y)
 *            追加到 Panel 进行中的多边形;
 *   右键   → 结束当前多边形;
 *   Esc    → 结束当前多边形;
 *   Backspace → 撤销上一个顶点。
 *
 * 投影用 rviz_rendering::ViewportProjectionFinder::getViewportPointProjectionOnXYPlane
 * (与内置 PoseTool / GoalTool 同一套 Jazzy API), 故无需自己摆弄 Ogre 射线。
 */

#include <memory>

#include <QObject>  // NOLINT (include order: Qt 头需在 Ogre 之外)

#include "rviz_common/tool.hpp"

namespace rviz_rendering
{
class ViewportProjectionFinder;
}  // namespace rviz_rendering

namespace sentry_trav_rviz_plugin
{

class TraversabilityTool : public rviz_common::Tool
{
    Q_OBJECT

public:
    TraversabilityTool();
    ~TraversabilityTool() override;

    void onInitialize() override;
    void activate() override;
    void deactivate() override;

    int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
    int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

private:
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};

}  // namespace sentry_trav_rviz_plugin
