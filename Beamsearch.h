#include "pgl_functs.hpp"
#include "RI.hpp"
#include "tinyxml2.hpp"
#include "cgal.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace PGL;
using namespace PPGL;

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
    double boxx, boxy;
    double calculateScore(const std::vector<Vector2d1>& polygons);
    bool doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2);
    Vector2d1 translatePolygon(const Vector2d1& polygon, double dx, double dy);
    std::vector<Vector2d1> beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect);
    void work(int beamWidth);
    int run();
    void get_more_geometry();
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
        CGAL_2D_Polygon_Area;

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

std::vector<Vector2d1> Beamsearch::beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>, greater<Candidate>> candidates;

    // 初始化一个空的候选解决方案集合
    candidates.push(Candidate({}, 0.0, {}));

    // 按照一定的优先放置顺序对输入的多边形进行排序，这里采用了按照面积从大到小排序的示例
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    //std::sort(sortedPolygons.begin(), sortedPolygons.end(), [](const Vector2d1& poly1, const Vector2d1& poly2) {
    //    // 优先放置顺序进行比较，可以按照面积、边长等进行排序
    //    return std::abs(poly1.area()) > std::abs(poly2.area());
    //    });

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
                if (nextCandidates.size() > beamWidth) {
                    cout << "被扔出的" << nextCandidates.top().score << endl;
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }
    // 获取最佳的候选解决方案
    return candidates.top().polygons;
}

void Beamsearch::work(int beamWidth) {
    Vector2d1 boundingRect;
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
    geometry_layer_output(bestSolution);
}

int Beamsearch::run()
{
    get_more_geometry();
    work(3);
    /*geometry_layer_output();*/
    return 0;
}

void Beamsearch::get_more_geometry()
{
    std::cout << "请输入处理原件的垂直截面的x，y" << endl;//如问题
    cin >> boxx >> boxy;
    std::cout << "要加入几个几何结构类型或者你要加入几个结构文件" << endl;
    int n;
    cin >> n;
    while (n--)
    {
        get_points_to_polygon();
    }
}

void Beamsearch::get_points_to_polygon() {
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
            std::cout << "Polygon Vertices:" << endl;
            for (auto it = points.begin(); it != points.end(); ++it)
                std::cout << "(" << (*it).x << ", " << (*it).y << ")" << endl;
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
    std::vector<Vector2d1> ans;
    vector<int> tt;
    std::cout << "图层加入加几个图" << endl;
    int n; cin >> n;
    while (n--) {
        std::cout << "加入的几何结构类型为：" << endl;
        int type;
        cin >> type;
        if (type > polygons.size()) {
            std::cout << "没有该类型的几何结构哦" << endl;
            n++;
            continue;
        }
        Vector2d1 temp = polygons[type - 1];
        ans.push_back(temp);
        tt.push_back(type);
    }
    std::cout << "成功, 目前加入有：" << endl;
    for (auto pt : tt) {
        cout << "种类型为" << pt << " 的元件" << endl;
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