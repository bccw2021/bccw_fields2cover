#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/intersection_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Polygon_2<K> Polygon_2;

int main() {
    // 定义线段
    Point_2 start(3.54188, -26.0721);
    Point_2 end(20.5419, -32.8303);
    Segment_2 segment(start, end);

    // 定义多边形
    std::vector<Point_2> polygon_points = {
        Point_2(7.223, -26.933),
        Point_2(6.698, -27.724),
        Point_2(-6.254, -19.486),
        Point_2(-5.891, -18.811),
        Point_2(7.223, -26.933)
    };
    Polygon_2 polygon(polygon_points.begin(), polygon_points.end());

    // 检查是否相交
    bool intersects = false;

    // 检查线段是否与多边形的任何边相交
    for (auto edge_it = polygon.edges_begin(); edge_it != polygon.edges_end(); ++edge_it) {
        Segment_2 edge = *edge_it;
        if (CGAL::do_intersect(segment, edge)) {
            intersects = true;
            break;
        }
    }

    // 如果没有边相交，检查线段的端点是否在多边形内部或边界上
    if (!intersects) {
        if (polygon.bounded_side(start) != CGAL::ON_UNBOUNDED_SIDE ||
            polygon.bounded_side(end) != CGAL::ON_UNBOUNDED_SIDE) {
            intersects = true;
        }
    }

    std::cout << "CGAL - Intersects: " << intersects << std::endl;
    return 0;
}
