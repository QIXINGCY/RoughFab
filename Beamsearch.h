#include "pgl_functs.hpp"
#include "RI.hpp"
#include "tinyxml2.hpp"
#include "cgal.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Filtered_kernel/internal/Static_filters/Do_intersect_2.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Constrained_Delaunay_triangulation_2<K> CDT;

using namespace std;
using namespace PGL;
using namespace PPGL;

double boxx, boxy;

double pointToPolygonDist(const Point_2& p, const Polygon_2& polygon) {
    int count = 0;
    double minDist = INFINITY;

    for (auto e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
        const Point_2& a = e->source();
        const Point_2& b = e->target();

        // 判断点 p 与线段 ab 是否在同一水平线上，并且 p 在 ab 的左侧
        if ((a.y() > p.y() != b.y() > p.y()) &&
            (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())) {
            count++;
        }

        // 计算点 p 到线段 ab 的最短距离，并更新最小距离
        Segment_2 s(a, b);
        minDist = std::min(minDist, squared_distance(p, s));
    }

    if (count % 2 == 0) {
        return std::sqrt(abs(minDist));

    }

    return -std::sqrt(abs(minDist));
}

void CGAL_2D_Polygon_Dart_Sampling(const vector<Polygon_2>& py, const double& d, vector<Point_2>& sampling_points, const int& total_iter)
{
    Functs::MAssert(d > 0 && d < 1.0, "CGAL_2D_Polygon_Dart_Sampling if (!(d > 0 && d < 1.0))");

    double xmin = 0;
    double ymin = 0;
    double xmax = boxx;
    double ymax = boxy;
    double diagonal_length = sqrt(boxx * boxx + boxy * boxy);
    double minimal_d = d * diagonal_length;
    
    
    int run = 0;
    vector<Point_2> insert_points;
    while (run < total_iter)
    {
        run++;
        double x = rand() / double(RAND_MAX);
        double y = rand() / double(RAND_MAX);
        x = (xmax - xmin) * x + xmin;
        y = (ymax - ymin) * y + ymin;
        Point_2 point(x, y);
        double distance = CGAL_IA_MAX_DOUBLE;
        for (int num = 0; num < py.size(); num++) {
            Polygon_2 poly=py[num];
            distance = min(distance, pointToPolygonDist(point, poly));
            
        }
        for (int num = 0; num < insert_points.size(); num++) {
            distance = min(distance, squared_distance(point, insert_points[num]));

        }
        if (distance > minimal_d)
        {
            insert_points.push_back(Point_2(x, y));
            run = 0;
        }
    }
    sampling_points = insert_points;
}



struct Candidate {
    std::vector<Vector2d1> polygons;  // 解决方案中的多边形集合
    double score;                    // 解决方案的得分
    std::set<int> typenum;          //解决方案中多边形序号
    // 构造函数
    Candidate(const std::vector<Vector2d1>& polys, double sc, const std::set<int>& tn) : polygons(polys), score(sc), typenum(tn) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //排序
    }
    bool operator>(const Candidate& other) const {
        return score > other.score; //排序
    }
};

class Beamsearch {
public:
    vector<Vector2d1> polygons;
    double calculateScore(const std::vector<Vector2d1>& polygons);
    bool doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2);
    Vector2d1 translatePolygon(const Vector2d1& polygon, double dx, double dy);
    std::vector<Vector2d1> beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect);
    void work();
    void get_points_to_polygon();
    std::vector<Vector2d1> perior_geometry_put();
    void geometry_layer_output(vector<Vector2d1> a);
};

double Beamsearch::calculateScore(const std::vector<Vector2d1>& polygons) {
    double totalArea = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    double minX = 100000000.0;
    double minY = 100000000.0;
    for (const Vector2d1 poly : polygons) {
        double need_area = PL().CGAL_2D_Polygon_Area_C(poly);
        totalArea += std::abs(need_area);
        for (const auto& vertex : poly) {
            maxY = std::max(maxY, vertex.y);
            maxX = std::max(maxX, vertex.x);
            minY = std::min(minY, vertex.y);
            minX = std::min(minX, vertex.x);
        }
    }
    // 计算解决方案中所有多边形面积除以总面积获得的面积占比作为评分
    double maxYScaledArea = (maxY - minY) * (maxX - minX);
    return totalArea / maxYScaledArea;
}

bool Beamsearch::doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2) {//碰撞检测，多边形求交
    for (const Vector2d1& one_polygon : poly2) {
        if (PL().CGAL_2D_Two_Polygons_Intersection_C(poly1, one_polygon) > 0) {
            return true; // 发生碰撞
        }
    }
    return false; // 未发生碰撞
}

Vector2d1 Beamsearch::translatePolygon(const Vector2d1& polygon, double dx, double dy) {
    std::vector<Vector2d> translatedVertices;
    // 遍历所有顶点，对每个顶点进行平移操作，并添加到新的顶点列表中
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {

        Vector2d translatedPoint((*it).x + dx, (*it).y + dy);
        translatedVertices.push_back(translatedPoint);
    }
    // 使用新的顶点列表构造一个新的Vector2d1对象并返回
    return Vector2d1(translatedVertices.begin(), translatedVertices.end());
}

bool beamssort(const Vector2d1& poly1, const Vector2d1& poly2)
{
    double xmax_score1, xmin_score1, xlength_score1, ylength_score1, area_score1;
    double xmax_score2, xmin_score2, xlength_score2, ylength_score2, area_score2;
    Vector2d max1, min1, max2, min2;
    Functs::GetBoundingBox(poly1, min1, max1);
    Functs::GetBoundingBox(poly2, min2, max2);
    xmax_score1 = max1.x / boxx;
    xmax_score2 = max2.x / boxx;
    xmin_score1 = min1.x / boxx;
    xmin_score2 = min2.x / boxx;
    xlength_score1 = (max1.x - min1.x) / boxx;
    xlength_score2 = (max2.x - min2.x) / boxx;
    ylength_score1 = (max1.y - min1.y) / boxy;
    ylength_score2 = (max2.y - min2.y) / boxy;
    area_score1 = PL().CGAL_2D_Polygon_Area_C(poly1) / (boxx * (max1.y - min1.y));
    area_score2 = PL().CGAL_2D_Polygon_Area_C(poly2) / (boxx * (max2.y - min2.y));

    double score1 = xmax_score1 + xmin_score1 + xlength_score1 + ylength_score1 + area_score1;
    double score2 = xmax_score2 + xmin_score2 + xlength_score2 + ylength_score2 + area_score2;
    return score1 > score2;
}

std::vector<Vector2d1> Beamsearch::beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>, greater<Candidate>> candidates;

    // 初始化一个空的候选解决方案集合
    candidates.push(Candidate({}, 0.0, {}));

    // 按照一定的优先放置顺序对输入的多边形进行排序，这里采用了按照面积从大到小排序的示例
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), beamssort);

    // 处理每个待放置的位置
    for (int times = 0; times < sortedPolygons.size(); times++) {
        std::priority_queue < Candidate, std::vector<Candidate>, greater<Candidate>> nextCandidates;

        // 处理当前轮次的每个候选解决方案
        while (!candidates.empty())
        {
            Candidate candidate = candidates.top();
            candidates.pop();
            int tab = 0;
            // 尝试在当前位置放置不同的多边形
            for (int i = 0; i < sortedPolygons.size(); i++)
            {
                // 如果多边形已经放置在解决方案中，则跳过
                if (candidate.typenum.count(i) != 0)
                {
                    continue;
                }
                if (tab < 3)tab++;
                else break;
                //将多边形放置在容器的最顶上
                Vector2d bomin, bomax, somin, somax;
                Functs::GetBoundingBox(boundingRect, bomin, bomax);
                Functs::GetBoundingBox(sortedPolygons[i], somin, somax);
                double dx = 0.0;
                double dy = bomax.y - somax.y;
                Vector2d1 finalPolygon = translatePolygon(sortedPolygons[i], dx, dy);
                if (doPolygonsCollide2(finalPolygon, candidate.polygons) || somax.y > boxy) {
                    tab--;
                    continue;
                }
                // 使用二分法进行平移，直到发生碰撞
                Functs::GetBoundingBox(finalPolygon, bomin, bomax);
                double bottom_distance = bomin.y;


                bool judge = 1;
                double pymin = bomin.y;
                while (bottom_distance > 10)
                {
                    pymin -= bottom_distance;
                    if (pymin < 0) {
                        cout << "发生边界碰撞" << endl;
                        pymin += bottom_distance;
                        bottom_distance /= 2.0;
                        continue;
                    }
                    Vector2d1 translatedPolygon = translatePolygon(finalPolygon, 0.0, -bottom_distance);
                    judge = doPolygonsCollide2(translatedPolygon, candidate.polygons);
                    if (judge == true)
                    {
                        pymin += bottom_distance;
                        bottom_distance /= 2.0;
                    }
                    else
                    {
                        finalPolygon = translatedPolygon;
                    }
                }
                // 生成新的候选解决方案
                std::vector<Vector2d1> newPolygons = candidate.polygons;
                std::set<int> temp = candidate.typenum;
                temp.insert(i);
                newPolygons.push_back(finalPolygon);

                // 计算新的解决方案的得分
                double newScore = calculateScore(newPolygons);
                cout << newScore << endl;//输出评分
                // 将新的解决方案添加到候选集合中
                geometry_layer_output(newPolygons);
                nextCandidates.push(Candidate(newPolygons, newScore, temp));
                // 保持候选队列的大小不超过束宽度
                while (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }
    // 获取最佳的候选解决方案
    return candidates.top().polygons;
}

void Beamsearch::work() {
    get_points_to_polygon();
    Vector2d1 boundingRect;
    int beamWidth = 3;
    boundingRect.push_back(Vector2d(0, 0));
    boundingRect.push_back(Vector2d(boxx, 0));
    boundingRect.push_back(Vector2d(boxx, boxy));
    boundingRect.push_back(Vector2d(0, boxy));
    std::vector<Vector2d1> wtf = perior_geometry_put();
    std::vector<Vector2d1> bestSolution = beamSearch(wtf, beamWidth, boundingRect);

    // 输出最佳解决方案
    std::cout << "最佳解决方案：" << std::endl;
    for (const Vector2d1& polygon : bestSolution) {
        // 输出多边形的坐标
        for (const Vector2d& point : polygon) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }
    if (bestSolution.empty())cout << "无法生成解决方案！" << endl;
    else geometry_layer_output(bestSolution);
}

void Beamsearch::get_points_to_polygon() {
    std::cout << "请输入处理原件的垂直截面的x，y" << endl;//如问题
    cin >> boxx >> boxy;
    std::cout << "如果你想加载已有的几何结构，请输入‘1’；你要创建新的几何结构，请输入‘2’:";
    int choose;
    cin >> choose;//choose决定你是要创建还是加载
    if (choose == 1) {
        std::cout << "请输入打开的文件是：";
        string address;
        cin >> address;
        ifstream infile;
        infile.open(address);
        if (!infile.is_open()) {
            std::cout << "文件打开失败" << endl;
            return;
        }
        string line;
        while (getline(infile, line)) {//每次从文件读取一行
            istringstream iss(line);
            Vector2d1 points;
            int n;
            iss >> n;
            double x, y;
            for (int i = 0; i < n; i++)
            {
                iss >> x >> y;
                points.push_back(Vector2d(x, y));
            }
            if (PL().CGAL_2D_Polygon_Is_Clockwise_Oriented_C(points))
            {
                std::reverse(points.begin(), points.end());
            }
            polygons.push_back(points);
        }

        infile.close();
    }
    else if (choose == 2) {//和上面很相像
        Vector2d1 points;
        std::cout << "请输入几何体点数:";
        int n;
        cin >> n;
        double x, y;
        for (int i = 0; i < n; i++)
        {
            cin >> x >> y;
            points.push_back(Vector2d(x, y));
        }
        if (PL().CGAL_2D_Polygon_Is_Clockwise_Oriented_C(points))
        {
            std::reverse(points.begin(), points.end());
        }
        std::cout << "Polygon Vertices:" << endl;
        for (auto it = points.begin(); it != points.end(); ++it)
            std::cout << "(" << (*it).x << ", " << (*it).y << ")" << endl;

        polygons.push_back(points);
    }
    else {
        std::cout << "输入模式错误！" << endl;
    }
}

std::vector<Vector2d1> Beamsearch::perior_geometry_put()
{
    std::vector<Vector2d1> ans = polygons;
    std::cout << "图形种类加入是否重复" << endl;
    bool ques;
    cin >> ques;
    if (ques) {
        std::cout << "重复的有几个" << endl;
        int n; cin >> n;
        while (n--) {
            int type;
            cin >> type;
            if (type > polygons.size()) {
                std::cout << "没有该类型的几何结构哦，请重新输入" << endl;
                n++;
                continue;
            }
            Vector2d1 temp = polygons[type - 1];
            ans.push_back(temp);
        }
    }
    return ans;
}

void Beamsearch::geometry_layer_output(vector<Vector2d1> a) {
    // 计算图像的尺寸

    // 创建一个黑色的图像
    cv::Mat rightimage(boxy, boxx / 2, CV_64FC3, cv::Scalar(0, 0, 0));

    // 绘制多边形
    for (const auto& polygon : a)
    {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon)
        {
            int x = vertex.x;
            int y = boxy - vertex.y;
            cv::Point point(x, y);
            points.push_back(point);
        }
        const cv::Point* pts = points.data();
        int num_points = points.size();
        cv::polylines(rightimage, &pts, &num_points, 1, true, cv::Scalar(255, 255, 255), 2);
    }
    cv::Mat leftimage;
    cv::flip(rightimage, leftimage, 1);
    cv::Mat symmetric_image;
    cv::hconcat(leftimage, rightimage, symmetric_image);
    cv::Point point1(boxx / 2, boxy);
    cv::Point point2(boxx / 2, 0);
    cv::line(symmetric_image, point1, point2, cv::Scalar(0, 0, 255), 1);
    // 显示图像
    cv::imshow("Polygons", symmetric_image);
    cv::waitKey(0);
    return;
}