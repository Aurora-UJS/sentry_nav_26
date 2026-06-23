#include "sentry_trav_rviz_plugin/annotation_bridge.hpp"

namespace sentry_trav_rviz_plugin
{

AnnotationBridge * AnnotationBridge::instance()
{
    // Meyers 单例: 线程安全的惰性构造, 进程退出时析构。
    static AnnotationBridge inst;
    return &inst;
}

}  // namespace sentry_trav_rviz_plugin
