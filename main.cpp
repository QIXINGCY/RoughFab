// main.cpp : Defines the entry point for the console application.
//


#include "pgl_functs.hpp"
#include "RI.hpp"
#include "tinyxml2.hpp"
#include "cgal.h"
#include <bits/stdc++.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace PGL;
using namespace PPGL;


//#include <sys/stat.h>
//#include <string>
//#include <fstream>



int Check2DIntersection()
{
	auto Load = [](Vector2d2& contours)
	{
		string address = "E:/Code/RoughFab/build/test.txt";
		ifstream infile;
		infile.open(address);
		if (!infile.is_open()) {
			std::cout << "文件打开失败" << endl;
			return;
		}

		string line;

		while (getline(infile, line))
		{//每次从文件读取一行
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
			contours.push_back(points);
		}
	};

	Vector2d2 contours;
	Load(contours);


	for (int i = 0; i < contours.size(); i++)
	{
		if (PL().CGAL_2D_Polygon_Is_Clockwise_Oriented_C(contours[i]))
		{
			std::reverse(contours[i].begin(), contours[i].end());
		}
		//Functs::Export_Segment(contours[i],);

		Vector2d center = Functs::GetCenter(contours[i]);

		for (int j = 0; j < contours[i].size(); j++)
		{
			contours[i][j] = contours[i][j] - center;
		}

		auto c3d = Functs::Vector2d3d(contours[i], 0.0);
		Functs::OutputObj3d("D:\\debug_" + Functs::IntString(i) + ".obj", c3d);

		bool b = PL().CGAL_2D_Polygon_Simple_C(contours[i]);

		std::cerr << "Simple test: " << i << " " << b << std::endl;
	}

	Vector1d1 des;
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = i + 1; j < contours.size(); j++)
		{
			double ds = PL().CGAL_2D_Distance_Polygon_Polygon_C(contours[i], contours[j]);
			//double cs = CGAL_2D_Intersection_Polygon_Polygon(contours[i], contours[j]);

			double cs = PL().CGAL_2D_Two_Polygons_Intersection_C(contours[i], contours[j]);

			std::cerr << i << " " << j << " dis: " << ds << " collision: " << cs << std::endl;
			des.push_back(ds);
		}
	}

	return 0;
}


struct perior_geometry {//集合类型，包含种类，还有x、y偏移量
    int type;
    int yadd;
};
double boxx, boxy;//全局变量，代表构型的x、y值

Vector2d1 a = { Vector2d(0,0), Vector2d(1,0), Vector2d(1,1)};
Vector2d2 b = {a,a,a};

vector<Polygon_2> polygons;//几何结构
vector<Polygon_2> geometry_layer;//几何图层
vector<perior_geometry> pgqueue;

void get_more_geometry();//循环下面这个函数
void get_points_to_polygon();//获得几何结构
bool judge_polygons_collision(Polygon_2 polygon_target);//碰撞判断
void get_geometry_layer();//几何结构加入图层并排布
void perior_geometry_put();
void geometry_layer_output(vector<Polygon_2> a);//图层输出
bool cmd(perior_geometry first, perior_geometry second);

// 结构体用于表示候选解决方案
struct Candidate {
    std::vector<Polygon_2> polygons;  // 解决方案中的多边形集合
    double score;                    // 解决方案的得分
    std::set<int> typenum;          //解决方案中多边形序号
    // 构造函数
    Candidate(const std::vector<Polygon_2>& polys, double sc, const std::set<int>& tn) : polygons(polys), score(sc), typenum(tn) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //排序
    }
};

// 计算候选解决方案的得分
double calculateScore(const std::vector<Polygon_2>& polygons) {
    double totalArea = 0.0;
    double maxX = 0.0;
    double maxY = 0.0;
    double minX = 100000000.0;
    double minY = 100000000.0;
    for (const Polygon_2& poly : polygons) {
        totalArea += std::abs(poly.area());
        for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
            maxY = std::max(maxY, it->y());
            maxX = std::max(maxX, it->x());
            minY = std::min(minY, it->y());
            minX = std::min(minX, it->x());
        }
    }
    // 计算解决方案中所有多边形面积除以总面积获得的面积占比作为评分
    double maxYScaledArea = (maxY - minY) * (maxX - minX);
    return totalArea / maxYScaledArea;
}

// 检查两个多边形是否发生碰撞（重叠）
bool doPolygonsCollide(const Polygon_2& poly1, const Polygon_2& poly2) {
    return CGAL::do_intersect(poly1, poly2);
}
// 检查一个多边形和一个多边形组是否发生碰撞（重叠）
bool doPolygonsCollide2(const Polygon_2& poly1, const vector<Polygon_2>& poly2) {//碰撞检测，多边形求交
    for (const Polygon_2& one_polygon : poly2) {
        if (CGAL::do_intersect(one_polygon, poly1)) {
            return true; // 发生碰撞
        }
    }
    return false; // 未发生碰撞
}

// 平移Polygon_2中的所有点坐标并返回新的多边形
Polygon_2 translatePolygon(const Polygon_2& polygon, double dx, double dy) {
    std::vector<Point_2> translatedVertices;
    // 遍历所有顶点，对每个顶点进行平移操作，并添加到新的顶点列表中
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        Point_2 translatedPoint(it->x() + dx, it->y() + dy);
        translatedVertices.push_back(translatedPoint);
    }
    // 使用新的顶点列表构造一个新的Polygon_2对象并返回
    return Polygon_2(translatedVertices.begin(), translatedVertices.end());
}

// 执行束搜索算法，在不同位置放置多边形
std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>> candidates;

    // 初始化一个空的候选解决方案集合
    candidates.push(Candidate({}, 0.0, {}));

    // 按照一定的优先放置顺序对输入的多边形进行排序，这里采用了按照面积从大到小排序的示例
    std::vector<Polygon_2> sortedPolygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), [](const Polygon_2& poly1, const Polygon_2& poly2) {
        // 优先放置顺序进行比较，可以按照面积、边长等进行排序
        return std::abs(poly1.area()) > std::abs(poly2.area());
        });

    // 处理每个待放置的位置
    for (int times = 0; times < inputPolygons.size(); times++) {
        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;

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
                if (tab < 1)tab++;
                else break;
                //已经放置的多边形最大maxY
                /*double maxY = 0.0;
                for (const Polygon_2& poly : candidate.polygons) {
                    for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
                        maxY = std::max(maxY, it->y());
                    }
                }*/

                //将多边形放置在容器的最顶上
                double dx = 0.0;
                double dy = boundingRect.bbox().ymax() - sortedPolygons[i].bbox().ymax();
                Polygon_2 finalPolygon = translatePolygon(sortedPolygons[i], dx, dy);
                // 使用二分法进行平移，直到发生碰撞
                double bottom_distance = finalPolygon.bbox().ymin();


                bool judge = 1;
                double pymin = finalPolygon.bbox().ymin();
                while (bottom_distance > 1)
                {
                    pymin -= bottom_distance;
                    if (pymin < 0) {
                        cout << "发生边界碰撞" << endl;
                        pymin += bottom_distance;
                        bottom_distance /= 2.0;
                        continue;
                    }
                    Polygon_2 translatedPolygon = translatePolygon(finalPolygon, 0.0, -bottom_distance);
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


                //double down = 0.0;
                //double up = dy;
                //int nnn = 0;
                //while (down < up) {
                //    nnn++;
                //    cout << nnn << "次" << "     " << down << "    " << up <<endl;
                //    double mid = (up + down) / 2.0;
                //    
                //    cout << "正在待加入" << i << endl;
                //    Polygon_2 translatedPolygon = translatePolygon(placedPolygon, 0.0, -mid);
                //    for (const int si : candidate.typenum) {
                //        cout << "种类" << si << "   ";
                //    }
                //    cout << endl;
                //    bool judge = doPolygonsCollide2(translatedPolygon, candidate.polygons);
                //    if (judge) 
                //    {
                //        down = mid + 1e-9; // 碰撞发生，向上收缩下边界
                //        cout << "发生碰撞" << endl;
                //    }
                //    else 
                //    {
                //        up = mid - 1e-9; // 无碰撞，向下收缩上边界
                //        cout << "没发生碰撞" << endl;
                //    }
                //}
                //// 回退到最后一次无碰撞的位置
                //double finalY = (up + down) / 2.0;
                //


                // 生成新的候选解决方案
                std::vector<Polygon_2> newPolygons = candidate.polygons;
                std::set<int> temp = candidate.typenum;
                temp.insert(i);
                newPolygons.push_back(finalPolygon);

                // 计算新的解决方案的得分
                double newScore = calculateScore(newPolygons);
                cout << newScore << endl;//输出评分
                // 将新的解决方案添加到候选集合中
                nextCandidates.push(Candidate(newPolygons, newScore, temp));
                // 保持候选队列的大小不超过束宽度
                if (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }

    // 获取最佳的候选解决方案
    return candidates.top().polygons;
}

void work(int beamWidth) {
    vector<Point_2> points;
    points.push_back(Point_2(0, 0));
    points.push_back(Point_2(boxx, 0));
    points.push_back(Point_2(boxx, boxy));
    points.push_back(Point_2(0, boxy));

    Polygon_2 boundingRect(points.begin(), points.end());

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
    geometry_layer_output(bestSolution);
}

int main()
{
    get_more_geometry();
    //get_geometry_layer();
    work(1);
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
            double x, y;
            for (int i = 0; i < n; i++)
            {
                iss >> x >> y;
                points.push_back(Point_2(x, y));
            }
            //polygons_xy_information xy;
            //iss >> xy.xmax >> xy.xmin >> xy.ymax >> xy.ymin;//x,y最大值最小值
            Polygon_2 polygontemp(points.begin(), points.end());
            //xy.type = polygons.size() + 1;
            //pxyinformation.push_back(xy);
            polygons.push_back(polygontemp);

            //std::cout << "几何图形生成成功,类型为" << xy.type << endl;
            //std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
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
        double x, y;
        for (int i = 0; i < n; i++)
        {
            cin >> x >> y;
            points.push_back(Point_2(x, y));
        }
        Polygon_2 polygontemp(points.begin(), points.end());


        /*polygons_xy_information xy;
        xy.type = polygons.size() + 1;
        xy.xmax = xy.xmin = x;
        xy.ymax = xy.ymin = y;
        for (auto it = points.begin(); it != points.end(); it++)
        {
            xy.xmax = max((int)(*it).x(), xy.xmax);
            xy.xmin = min((int)(*it).x(), xy.xmin);
            xy.ymax = max((int)(*it).x(), xy.ymax);
            xy.ymin = min((int)(*it).x(), xy.ymin);
        }*/

        //std::cout << "几何图形生成成功,类型为" << xy.type << endl;
        //std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        std::cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        //pxyinformation.push_back(xy);


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

void geometry_layer_output(vector<Polygon_2> a) {
    // 计算图像的尺寸

    // 创建一个黑色的图像
    cv::Mat image(boxy + 10, 2 * boxx + 10, CV_64FC3, cv::Scalar(0, 0, 0));

    // 绘制多边形
    for (const auto& polygon : a)
    {
        std::vector<cv::Point> points;
        for (const auto& vertex : polygon.vertices())
        {
            double x = vertex.x() + image.cols / 2;
            double y = image.rows - vertex.y() - 10;
            cv::Point point(x, y);
            points.push_back(point);
        }
        const cv::Point* pts = points.data();
        int num_points = points.size();
        cv::polylines(image, &pts, &num_points, 1, true, cv::Scalar(255, 255, 255), 1);
    }
    cv::Point point1(image.cols / 2, image.rows);
    cv::Point point2(image.cols / 2, 0);
    cv::line(image, point1, point2, cv::Scalar(0, 0, 255), 1);
    // 显示图像
    cv::imshow("Polygons", image);
    cv::waitKey(0);
    return;
}





//// 执行束搜索算法
//std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
//{
//    std::priority_queue < Candidate, std::vector<Candidate>> candidates;
//    // 初始化一个空解决方案
//    candidates.push(Candidate({}, 0.0));
//    // 处理每个输入多边形
//    for (const Polygon_2& polygon : inputPolygons)
//    {
//        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;
//
//        // 处理当前轮次的每个候选解决方案
//        while (!candidates.empty())
//        {
//            Candidate candidate = candidates.top();
//            candidates.pop();
//
//            // 在不同位置插入当前多边形，生成新的候选解决方案
//            for (int i = 0; i <= candidate.polygons.size(); ++i)
//            {
//                std::vector<Polygon_2> newPolygons = candidate.polygons;
//                newPolygons.insert(newPolygons.begin() + i, polygon);
//                // 检查解决方案中的多边形是否发生碰撞
//                bool hasCollisions = false;
//                for (int j = 0; j < newPolygons.size(); ++j)
//                {
//                    if (j != i && doPolygonsCollide(newPolygons[i], newPolygons[j]))
//                    {
//                        hasCollisions = true;
//                        break;
//                    }
//                }
//                if (hasCollisions)
//                {
//                    // 尝试在y轴上移动多边形来优化位置
//                    double currentScore = calculateScore(newPolygons);
//                    double newY = newPolygons[i].bbox().ymax();  // 当前位置的y坐标
//                    double stepSize = 0.01;  // 步长
//                    double bestY = newY;
//                    for (int k = 0; k < 10; ++k)
//                    {
//                        newY += stepSize; // 尝试在y轴上加上步长
//
//                        double newScore = calculateScore(newPolygons);
//                        // 如果得分更好，则保留新位置
//                        if (newScore > currentScore)
//                        {
//                            bestY = newY;
//                            currentScore = newScore;
//                        }
//                    }
//                }
//                else
//                {
//                    // 尝试在y轴上移动多边形来优化位置
//                    double currentScore = calculateScore(newPolygons);
//                    double newY = newPolygons[i].bbox().ymax();  // 当前位置的y坐标
//                    double stepSize = 0.01;  // 步长
//                    double bestY = newY;
//                    for (int k = 0; k < 10; ++k)
//                    {
//                        newY -= stepSize; // 尝试在y轴上减少步长
//
//                        double newScore = calculateScore(newPolygons);
//                        // 如果得分更好，则保留新位置
//                        if (newScore > currentScore)
//                        {
//                            bestY = newY;
//                            currentScore = newScore;
//                        }
//                    }
//                }
//                // 保持候选队列的大小不超过束宽度
//                if (nextCandidates.size() > beamWidth)
//                {
//                    nextCandidates.pop();
//                }
//            }
//        }
//        candidates = nextCandidates;
//    }
//    // 获取最佳的候选解决方案
//    std::vector<Polygon_2> bestSolution = candidates.top().polygons;
//    // 对最佳解决方案进行必要的后处理
//    return bestSolution;
//}