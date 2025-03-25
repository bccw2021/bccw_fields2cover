//#include "clipper2/clipper.h"
//
//int main() {
//    // 定义线段
//    Clipper2Lib::PathsD subject = {{
//        {3.54188, -26.0721},
//        {20.5419, -32.8303}
//    }};
//
//    // 定义多边形
//    Clipper2Lib::PathsD clip = {{
//        {7.223, -26.933},
//        {6.698, -27.724},
//        {-6.254, -19.486},
//        {-5.891, -18.811},
//        {7.223, -26.933}
//    }};
//
//    // 计算交集
//    Clipper2Lib::PathsD solution = Clipper2Lib::Intersect(subject, clip, Clipper2Lib::FillRule::NonZero);
//
//    // 检查是否有交集
//    bool intersects = !solution.empty();
//    std::cout << "Clipper2 - Intersects: " << intersects << std::endl;
//    return 0;
//}


//#include "clipper2/clipper.h"
//#include <iostream>
//
//int main() {
//    // 定义线段
//    Clipper2Lib::PathsD subject = {{
//        {3.54188, -26.0721},
//        {20.5419, -32.8303}
//    }};
//
//    // 定义多边形，确保闭合
//    Clipper2Lib::PathsD clip = {{
//        {7.223, -26.933},
//        {6.698, -27.724},
//        {-6.254, -19.486},
//        {-5.891, -18.811},
//        {7.223, -26.933}  // 确保闭合
//    }};
//
//    // 创建 Clipper 对象
//    Clipper2Lib::ClipperD clipper;
//    clipper.AddSubject(subject);  // 添加线段作为 subject
//    clipper.AddClip(clip);        // 添加多边形作为 clip
//
//    // 执行交集操作
//    Clipper2Lib::PathsD solution;
//    clipper.Execute(Clipper2Lib::ClipType::Intersection, Clipper2Lib::FillRule::NonZero, solution);
//
//    // 检查交集结果
//    bool intersects = !solution.empty();
//    std::cout << "Clipper2 - Intersects: " << intersects << std::endl;
//
//    // 额外检查：如果线段与多边形边界相交，也算相交
//    Clipper2Lib::PathsD clipped;
//    clipper.Execute(Clipper2Lib::ClipType::Difference, Clipper2Lib::FillRule::NonZero, clipped);
//    if (!clipped.empty() && clipped[0].size() > 0) {
//        intersects = true;  // 如果有剩余部分，说明有交集
//    }
//
//    std::cout << "Clipper2 - Final Intersects: " << intersects << std::endl;
//    return 0;
//}


//// OK
//#include "clipper2/clipper.h"
//#include <iostream>
//
//int main() {
//    // Define the line segment
//    Clipper2Lib::PathsD subject = {{
//        {3.54188, -26.0721},
//        {20.5419, -32.8303}
//    }};
//
//    // Define the polygon
//    Clipper2Lib::PathsD clip = {{
//        {7.223, -26.933},
//        {6.698, -27.724},
//        {-6.254, -19.486},
//        {-5.891, -18.811},
//        {7.223, -26.933}
//    }};
//
//    // Inflate the line segment to make it a thin polygon (width 0.01)
//    Clipper2Lib::PathsD subject_inflated = Clipper2Lib::InflatePaths(subject, 0.01, Clipper2Lib::JoinType::Square, Clipper2Lib::EndType::Square);
//
//    // Compute intersection
//    Clipper2Lib::PathsD solution = Clipper2Lib::Intersect(subject_inflated, clip, Clipper2Lib::FillRule::NonZero);
//
//    // Check if intersection is not empty
//    bool intersects = !solution.empty();
//    std::cout << "Clipper2 - Intersects: " << intersects << std::endl;
//
//    return 0;
//}


// 引入 Clipper2 库头文件（注意与 Clipper1 的路径不同）
#include "clipper2/clipper.h"
#include <iostream>  // 标准输入输出库

int main() {
    // =============== 1. 定义线段（开放路径） ===============
    // Clipper2Lib::PathsD 表示双精度浮点路径集合（PathD 的集合）
    Clipper2Lib::PathsD subject = {{
        {3.54188, -26.0721},   // 线段起点坐标
        {20.5419, -32.8303}    // 线段终点坐标
    }};  // 路径集合的嵌套大括号结构是 Clipper2 的规范用法

    // =============== 2. 定义多边形（闭合路径） ===============
    Clipper2Lib::PathsD clip = {{
        {7.223, -26.933},     // 多边形顶点1
        {6.698, -27.724},     // 顶点2
        {-6.254, -19.486},    // 顶点3
        {-5.891, -18.811},    // 顶点4
        {7.223, -26.933}      // 闭合多边形，回到起点
    }};  // 注意最后一个点与起点相同以闭合路径

    // =============== 3. 将线段膨胀为细长多边形 ===============
    // 因线段本身是零宽度，需通过膨胀生成微小宽度多边形才能进行面积交集检测
    Clipper2Lib::PathsD subject_inflated =
        Clipper2Lib::InflatePaths(
            subject,          // 原始路径
            0.01,             // 膨胀宽度（单位与坐标一致）
            Clipper2Lib::JoinType::Square,  // 连接处直角处理
            Clipper2Lib::EndType::Square    // 端点直角处理
        );  // 膨胀后的路径变成宽度为 0.01 的矩形条带

    // =============== 4. 计算交集 ===============
    // 使用 NonZero 填充规则检测实际覆盖区域
    Clipper2Lib::PathsD solution =
        Clipper2Lib::Intersect(
            subject_inflated,  // 膨胀后的线段（视为多边形）
            clip,              // 原始多边形
            Clipper2Lib::FillRule::NonZero  // 非零填充规则
        );  // 交集结果存储在 solution 中

    // =============== 5. 判断结果 ===============
    bool intersects = !solution.empty();  // 交集非空即为相交
    std::cout << "Clipper2 - Intersects: " << intersects << std::endl;

    return 0;
}
