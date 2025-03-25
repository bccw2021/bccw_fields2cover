//#include "clipper/clipper.h"
//
//int main() {
//    const double scale = 100000.0;  // 缩放因子
//
//    // 定义线段（缩放后）
//    ClipperLib::Paths subject;
//    subject.push_back(ClipperLib::Path{
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(3.54188 * scale), static_cast<ClipperLib::cInt>(-26.0721 * scale)),
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(20.5419 * scale), static_cast<ClipperLib::cInt>(-32.8303 * scale))
//    });
//
//    // 定义多边形（缩放后）
//    ClipperLib::Paths clip;
//    clip.push_back(ClipperLib::Path{
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(7.223 * scale), static_cast<ClipperLib::cInt>(-26.933 * scale)),
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(6.698 * scale), static_cast<ClipperLib::cInt>(-27.724 * scale)),
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(-6.254 * scale), static_cast<ClipperLib::cInt>(-19.486 * scale)),
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(-5.891 * scale), static_cast<ClipperLib::cInt>(-18.811 * scale)),
//        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(7.223 * scale), static_cast<ClipperLib::cInt>(-26.933 * scale))
//    });
//
//    // 创建 Clipper 对象
//    ClipperLib::Clipper c;
//    c.AddPaths(subject, ClipperLib::ptSubject, false);  // 线段是开放路径
//    c.AddPaths(clip, ClipperLib::ptClip, true);  // 多边形是闭合路径
//
//    // 计算交集
//    ClipperLib::Paths solution;
//    c.Execute(ClipperLib::ctIntersection, solution);
//
//    // 检查是否有交集
//    bool intersects = !solution.empty();
//    std::cout << "Clipper v1 - Intersects: " << intersects << std::endl;
//    return 0;
//}


#include "clipper/clipper.h"
#include <iostream>  // 添加缺失的标准库头文件

int main() {
    const double scale = 100000.0;  // 缩放因子

    // 定义线段（开放路径）
    ClipperLib::Paths subject;
    subject.push_back(ClipperLib::Path{
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(3.54188 * scale), 
                             static_cast<ClipperLib::cInt>(-26.0721 * scale)),
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(20.5419 * scale), 
                             static_cast<ClipperLib::cInt>(-32.8303 * scale))
    });

    // 定义多边形（闭合路径）
    ClipperLib::Paths clip;
    clip.push_back(ClipperLib::Path{
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(7.223 * scale), 
                             static_cast<ClipperLib::cInt>(-26.933 * scale)),
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(6.698 * scale), 
                             static_cast<ClipperLib::cInt>(-27.724 * scale)),
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(-6.254 * scale), 
                             static_cast<ClipperLib::cInt>(-19.486 * scale)),
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(-5.891 * scale), 
                             static_cast<ClipperLib::cInt>(-18.811 * scale)),
        ClipperLib::IntPoint(static_cast<ClipperLib::cInt>(7.223 * scale), 
                             static_cast<ClipperLib::cInt>(-26.933 * scale))
    });

    // 创建 Clipper 对象
    ClipperLib::Clipper c;
    c.AddPaths(subject, ClipperLib::ptSubject, false);  // 第三个参数: 是否闭合
    c.AddPaths(clip, ClipperLib::ptClip, true);

    // 计算交集并处理结果
    ClipperLib::Paths solution;
    bool success = c.Execute(ClipperLib::ctIntersection, 
                            solution, 
                            ClipperLib::pftNonZero,   // 填充规则
                            ClipperLib::pftNonZero);

    // 输出结果
    std::cout << "Clipper v1 - Intersects: " 
              << (success && !solution.empty()) 
              << std::endl;

    return 0;
}
