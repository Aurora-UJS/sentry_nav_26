#pragma once
/**
 * AnnotationBridge: Tool 与 Panel 之间的进程内信号桥 (单例)。
 *
 * 动机:
 *   pluginlib 分别实例化 TraversabilityTool 与 TraversabilityPanel, 两者互不持有
 *   对方指针。但它们运行在同一个 RViz 进程的同一个 GUI 线程里, 故用一个 QObject
 *   单例转发 Qt 信号即可解耦通信: Tool 在 3D 视图取到点 → 通过本桥发信号 →
 *   Panel 收点并刷新预览。
 */

#include <QObject>

namespace sentry_trav_rviz_plugin
{

class AnnotationBridge : public QObject
{
    Q_OBJECT

public:
    // 进程级单例 (首次调用时在 GUI 线程惰性构造)
    static AnnotationBridge * instance();

    // --- Tool 侧调用 (发射信号) ---
    void emitPoint(double x, double y) { Q_EMIT pointAdded(x, y); }  // 追加一个顶点
    void emitFinish() { Q_EMIT regionFinished(); }                   // 结束当前多边形
    void emitUndo() { Q_EMIT pointUndone(); }                        // 撤销上一个顶点

Q_SIGNALS:
    void pointAdded(double x, double y);
    void regionFinished();
    void pointUndone();

private:
    AnnotationBridge() = default;
    ~AnnotationBridge() override = default;
    AnnotationBridge(const AnnotationBridge &) = delete;
    AnnotationBridge & operator=(const AnnotationBridge &) = delete;
};

}  // namespace sentry_trav_rviz_plugin
