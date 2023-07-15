#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include<bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <opencv2/opencv.hpp>
#include"build/search_core/beam_search.h"

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;//点
typedef CGAL::Polygon_2<K> Polygon_2;

struct perior_geometry {//集合类型，包含种类，还有x、y偏移量
    int type;
    int yadd;
};
struct polygons_xy_information {//包围盒信息，最简单的包围盒，x、y最大最小值
    int type;
    int xmin, xmax, ymin, ymax;
};
int boxx, boxy;//全局变量，代表构型的x、y值

vector<Polygon_2> polygons;//几何结构
vector<Polygon_2> geometry_layer;//几何图层
vector<perior_geometry> pgqueue;
vector<polygons_xy_information> pxyinformation;//包围盒队列

void get_more_geometry();//循环下面这个函数
void get_points_to_polygon();//获得几何结构
bool judge_polygons_collision(Polygon_2 polygon_target);//碰撞判断
void get_geometry_layer();//几何结构加入图层并排布
void perior_geometry_put();
polygons_xy_information find_type_xy(int type);
void geometry_layer_output();//图层输出
bool cmd(perior_geometry first, perior_geometry second);

// 结构体用于表示候选解决方案
struct Candidate {
    std::vector<Polygon_2> polygons;  // 解决方案中的多边形集合
    double score;                    // 解决方案的得分
    // 构造函数
    Candidate(const std::vector<Polygon_2>& polys, double sc) : polygons(polys), score(sc) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; // 从大到小排序
    }
};

// 计算候选解决方案的得分
double calculateScore(const std::vector<Polygon_2>& polygons) {
    // 根据具体问题定义评分函数
    // 例如，可以计算多边形覆盖的总面积
    double totalArea = 0.0;
    for (const Polygon_2& polygon : polygons) {
        totalArea += polygon.area();
    }
    return totalArea;
}

// 检查两个多边形是否发生碰撞（重叠）
bool doPolygonsCollide(const Polygon_2& poly1, const Polygon_2& poly2) {
    if (CGAL::do_intersect(poly1, poly2)) {
        return true; // 发生碰撞
    }
    return false;
}

// 平移Polygon_2中的所有点坐标
void translatePolygon(Polygon_2& polygon, double dx, double dy) {
    // 遍历所有顶点，对每个顶点进行平移操作

}

// 执行束搜索算法
std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>> candidates;
    // 初始化一个空解决方案
    candidates.push(Candidate({}, 0.0));
    // 处理每个输入多边形
    for (const Polygon_2& polygon : inputPolygons)
    {
        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;

        // 处理当前轮次的每个候选解决方案
        while (!candidates.empty())
        {
            Candidate candidate = candidates.top();
            candidates.pop();

            // 在不同位置插入当前多边形，生成新的候选解决方案
            for (int i = 0; i <= candidate.polygons.size(); ++i)
            {
                std::vector<Polygon_2> newPolygons = candidate.polygons;
                newPolygons.insert(newPolygons.begin() + i, polygon);
                // 检查解决方案中的多边形是否发生碰撞
                bool hasCollisions = false;
                for (int j = 0; j < newPolygons.size(); ++j)
                {
                    if (j != i && doPolygonsCollide(newPolygons[i], newPolygons[j]))
                    {
                        hasCollisions = true;
                        break;
                    }
                }
                if (hasCollisions)
                {
                    // 尝试在y轴上移动多边形来优化位置
                    double currentScore = calculateScore(newPolygons);
                    double newY = newPolygons[i].bbox().ymax();  // 当前位置的y坐标
                    double stepSize = 0.01;  // 步长
                    double bestY = newY;
                    for (int k = 0; k < 10; ++k)
                    {
                        newY += stepSize; // 尝试在y轴上加上步长

                        double newScore = calculateScore(newPolygons);
                        // 如果得分更好，则保留新位置
                        if (newScore > currentScore)
                        {
                            bestY = newY;
                            currentScore = newScore;
                        }
                    }
                }
                else
                {
                    // 尝试在y轴上移动多边形来优化位置
                    double currentScore = calculateScore(newPolygons);
                    double newY = newPolygons[i].bbox().ymax();  // 当前位置的y坐标
                    double stepSize = 0.01;  // 步长
                    double bestY = newY;
                    for (int k = 0; k < 10; ++k)
                    {
                        newY -= stepSize; // 尝试在y轴上减少步长

                        double newScore = calculateScore(newPolygons);
                        // 如果得分更好，则保留新位置
                        if (newScore > currentScore)
                        {
                            bestY = newY;
                            currentScore = newScore;
                        }
                    }
                }
                // 保持候选队列的大小不超过束宽度
                if (nextCandidates.size() > beamWidth)
                {
                    nextCandidates.pop();
                }
            }
        }
        candidates = nextCandidates;
    }
    // 获取最佳的候选解决方案
    std::vector<Polygon_2> bestSolution = candidates.top().polygons;
    // 对最佳解决方案进行必要的后处理
    return bestSolution;
}

void work() {
    vector<Point_2> points;
    points.push_back(Point_2(0, 0));
    points.push_back(Point_2(boxx, 0));
    points.push_back(Point_2(boxx, boxy));
    points.push_back(Point_2(0, boxy));

    Polygon_2 boundingRect(points.begin(), points.end());

    int beamWidth = 3;
    std::vector<Polygon_2> bestSolution = beamSearch(polygons, beamWidth, boundingRect);

    // 输出最佳解决方案
    std::cout << "最佳解决方案：" << std::endl;
    for (const Polygon_2& polygon : bestSolution) {
        // 输出多边形的坐标
        for (const Point_2& point : polygon.container()) {
            std::cout << "(" << point.x() << ", " << point.y() << ") ";
        }
        std::cout << std::endl;
    }

}

int main()
{
    get_more_geometry();
    //get_geometry_layer();
    work();
    //geometry_layer_output();
    return 0;
}


void get_more_geometry()
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
void get_points_to_polygon() {
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
            vector<Point_2> points;
            int n;
            iss >> n;
            int x, y;
            for (int i = 0; i < n; i++)
            {
                iss >> x >> y;
                points.push_back(Point_2(x, y));
            }
            polygons_xy_information xy;
            iss >> xy.xmax >> xy.xmin >> xy.ymax >> xy.ymin;//x,y最大值最小值
            Polygon_2 polygontemp(points.begin(), points.end());
            xy.type = polygons.size() + 1;
            pxyinformation.push_back(xy);
            polygons.push_back(polygontemp);

            std::cout << "几何图形生成成功,类型为" << xy.type << endl;
            std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
            std::cout << "Polygon Vertices:" << endl;
            for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
                std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;
        }

        infile.close();
    }
    else if (choose == 2) {//和上面很相像
        vector<Point_2> points;
        std::cout << "请输入几何体点数:";
        int n;
        cin >> n;
        int x, y;
        for (int i = 0; i < n; i++)
        {
            cin >> x >> y;
            points.push_back(Point_2(x, y));
        }
        Polygon_2 polygontemp(points.begin(), points.end());


        polygons_xy_information xy;
        xy.type = polygons.size() + 1;
        xy.xmax = xy.xmin = x;
        xy.ymax = xy.ymin = y;
        for (auto it = points.begin(); it != points.end(); it++)
        {
            xy.xmax = max((int)(*it).x(), xy.xmax);
            xy.xmin = min((int)(*it).x(), xy.xmin);
            xy.ymax = max((int)(*it).x(), xy.ymax);
            xy.ymin = min((int)(*it).x(), xy.ymin);
        }

        std::cout << "几何图形生成成功,类型为" << xy.type << endl;
        std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        std::cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        pxyinformation.push_back(xy);


    }
    else {
        std::cout << "输入模式错误！" << endl;
    }
}


bool judge_polygons_collision(Polygon_2 polygon_target) {//碰撞检测，多边形求交
    for (const Polygon_2& polygon : geometry_layer) {
        if (CGAL::do_intersect(polygon, polygon_target)) {
            return true; // 发生碰撞
        }
    }
    return false; // 未发生碰撞
}

polygons_xy_information find_type_xy(int type)//判断是否存在这个图形序号
{
    for (auto it = pxyinformation.begin(); it != pxyinformation.end(); it++)
    {
        if ((*it).type == type)return *it;
    }
    polygons_xy_information shibai;
    shibai.type = -1;
    return shibai;
}

bool cmd(perior_geometry first, perior_geometry second) {
    polygons_xy_information first_xy = find_type_xy(first.type);
    polygons_xy_information second_xy = find_type_xy(second.type);
    if (first_xy.xmax != second_xy.xmax)//外边缘谁远谁放
        return first_xy.xmax >= second_xy.xmax;
    if (first_xy.xmax - first_xy.xmin != second_xy.xmax - second_xy.xmin)//长度长
        return first_xy.xmax - first_xy.xmin >= second_xy.xmax - second_xy.xmin;
    if (first_xy.ymax - first_xy.ymin != second_xy.ymax - second_xy.ymin)//高度长
        return first_xy.ymax - first_xy.ymin >= second_xy.ymax - second_xy.ymin;
    return first_xy.xmin >= second_xy.xmin;// 内边缘谁远谁放
}

void perior_geometry_put() {
    pgqueue.clear();
    std::cout << "图层加入加几个图" << endl;
    int n; cin >> n;
    while (n--) {
        perior_geometry temp;
        std::cout << "加入的几何结构类型为：" << endl;
        cin >> temp.type;
        temp.type--;
        if (temp.type > polygons.size()) {
            std::cout << "没有该类型的几何结构哦" << endl;
            n++;
            continue;
        }
        cin >> temp.yadd;
        pgqueue.push_back(temp);
    }
    std::cout << "成功" << endl;
    sort(pgqueue.begin(), pgqueue.end(), cmd);
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++)
    {
        std::cout << "类型" << (*itt).type + 1 << "原点改变量" << (*itt).yadd << endl;
    }
    return;
}

void get_geometry_layer() {
    perior_geometry_put();
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++) {

        Polygon_2 newone = polygons[(*itt).type];//准备加入的
        for (auto it = newone.vertices_begin(); it != newone.vertices_end(); ++it)//点循环
        {
            *it = Point_2(it->x(), it->y() + (*itt).yadd);//位移
        }
        if (judge_polygons_collision(newone)) {
            std::cout << "加入的几何图形与原有几何图形发生碰撞";
            continue;
        }
        else {
            std::cout << "几何图形生成成功";
            std::cout << "Polygon Vertices:" << endl;
            for (auto it = newone.vertices_begin(); it != newone.vertices_end(); ++it)
            {
                std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;
            }
            geometry_layer.push_back(newone);
        }
    }
}

void geometry_layer_output() {
    // 计算图像的尺寸
    int width = 1000;
    int height = 800;

    // 创建一个黑色的图像
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // 绘制多边形
    for (const auto& polygon : geometry_layer)
    {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon.vertices())
        {
            int x = vertex.x() + 100;
            int y = image.rows - vertex.y() - 100;
            cv::Point point(x, y);
            points.push_back(point);
        }
        const cv::Point* pts = points.data();
        int num_points = points.size();
        cv::polylines(image, &pts, &num_points, 1, true, cv::Scalar(255, 255, 255), 2);
    }
    cv::Point point1(100, image.rows - 100);
    cv::Point point2(100 + boxx, image.rows - 100);
    cv::Point point3(100 + boxx, image.rows - 100 - boxy);
    cv::Point point4(100, image.rows - 100 - boxy);
    cv::line(image, point1, point2, cv::Scalar(0, 0, 255), 1);
    cv::line(image, point2, point3, cv::Scalar(0, 0, 255), 1);
    cv::line(image, point3, point4, cv::Scalar(0, 0, 255), 1);
    cv::line(image, point4, point1, cv::Scalar(0, 0, 255), 1);
    // 显示图像
    cv::imshow("Polygons", image);
    cv::waitKey(0);
    return;
}






