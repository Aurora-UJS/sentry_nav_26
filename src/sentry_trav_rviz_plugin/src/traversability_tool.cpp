#include "sentry_trav_rviz_plugin/traversability_tool.hpp"

#include <utility>

#include <OgreVector.h>

#include <QKeyEvent>

#include "rviz_common/display_context.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/render_window.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "sentry_trav_rviz_plugin/annotation_bridge.hpp"

namespace sentry_trav_rviz_plugin
{

TraversabilityTool::TraversabilityTool()
{
    // 工具栏激活快捷键 (ToolManager 会把它显示在工具名后)
    shortcut_key_ = 'r';
}

TraversabilityTool::~TraversabilityTool() = default;

void TraversabilityTool::onInitialize()
{
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    setName("Traversability Annotate");
}

void TraversabilityTool::activate()
{
    setStatus("左键: 取点(投影到地面 z=0) | 右键/Esc: 结束多边形 | Backspace: 撤销上一点");
}

void TraversabilityTool::deactivate()
{
}

int TraversabilityTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
    // 左键按下: 把屏幕坐标投影到地面 XY 平面, 命中则追加顶点。
    if (event.leftDown()) {
        auto projection = projection_finder_->getViewportPointProjectionOnXYPlane(
            event.panel->getRenderWindow(), event.x, event.y);
        if (projection.first) {
            const Ogre::Vector3 & p = projection.second;
            AnnotationBridge::instance()->emitPoint(p.x, p.y);
        }
        return Render;
    }

    // 右键: 结束当前多边形。
    if (event.rightDown()) {
        AnnotationBridge::instance()->emitFinish();
        return Render;
    }

    return 0;
}

int TraversabilityTool::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * /*panel*/)
{
    if (event->key() == Qt::Key_Escape) {
        AnnotationBridge::instance()->emitFinish();
        return Render;
    }
    if (event->key() == Qt::Key_Backspace) {
        AnnotationBridge::instance()->emitUndo();
        return Render;
    }
    return 0;
}

}  // namespace sentry_trav_rviz_plugin

PLUGINLIB_EXPORT_CLASS(sentry_trav_rviz_plugin::TraversabilityTool, rviz_common::Tool)
