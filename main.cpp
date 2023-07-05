#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include <CGAL/Boolean_set_operations_2.h>
#include <opencv2/opencv.hpp>

using namespace std;

struct perior_geometry {
    int type;
    int xadd;//delete
    int yadd;
};
struct polygons_xy_information {
    int type;
    int xmin, xmax, ymin, ymax;
};
int boxx, boxy;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;

vector<Polygon_2> polygons;//几何结构
vector<Polygon_2> geometry_layer;//几何图层
vector<perior_geometry> pgqueue;
vector<polygons_xy_information> pxyinformation;

void get_more_geometry();
void get_points_to_polygon();//获得几何结构
bool judge_polygons_collision(Polygon_2 polygon_target);//碰撞判断
void get_geometry_layer();//几何结构加入图层并排布
void perior_geometry_put();
polygons_xy_information find_type_xy(int type);
void geometry_layer_output();//图层输出
bool cmd(perior_geometry first, perior_geometry second);

int main()
{
    get_more_geometry();
    get_geometry_layer();
    geometry_layer_output();
    return 0;
}

void get_more_geometry()
{
    cout << "请输入处理原件的垂直截面的x，y" << endl;
    cin >> boxx >> boxy;
    cout << "要加入几个几何结构类型" << endl;
    int n;
    cin >> n;
    while (n--)
    {
        get_points_to_polygon();
    }
}
void get_points_to_polygon() {
    cout << "如果你想加载已有的几何结构，请输入‘1’；你要创建新的几何结构，请输入‘2’:";
    int choose;
    cin >> choose;
    if (choose == 1) {
        cout << "请输入打开的文件是：";
        string address;
        cin >> address;
        ifstream infile;
        infile.open(address);
        if (!infile.is_open()) {
            cout << "文件打开失败" << endl;
            return;
        }
        vector<Point_2> points;
        int n;
        infile >> n;
        int x, y;
        for (int i = 0; i < n; i++)
        {
            infile >> x >> y;
            points.push_back(Point_2(x, y));
        }
        polygons_xy_information xy;
        infile >> xy.xmax >> xy.xmin >> xy.ymax >> xy.ymin;
        Polygon_2 polygontemp(points.begin(), points.end());
        xy.type = polygons.size() + 1;
        pxyinformation.push_back(xy);
        polygons.push_back(polygontemp);

        cout << "几何图形生成成功,类型为" << xy.type << endl;
        cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        infile.close();
    }
    else if (choose == 2) {
        vector<Point_2> points;
        cout << "请输入几何体点数:";
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

        cout << "几何图形生成成功,类型为" << xy.type << endl;
        cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        pxyinformation.push_back(xy);


    }
    else {
        cout << "输入模式错误！" << endl;
    }
}



bool judge_polygons_collision(Polygon_2 polygon_target) {
    for (const Polygon_2& polygon : geometry_layer) {
        if (CGAL::do_intersect(polygon, polygon_target)) {
            return true; // 发生碰撞
        }
    }
    return false; // 未发生碰撞
}

polygons_xy_information find_type_xy(int type)
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
    if (first_xy.xmax + first.xadd != second_xy.xmax + second.xadd)//外边缘谁远谁放
        return first_xy.xmax + first.xadd >= second_xy.xmax + second.xadd;
    if (first_xy.xmax - first_xy.xmin != second_xy.xmax - second_xy.xmin)//长度长
        return first_xy.xmax - first_xy.xmin >= second_xy.xmax - second_xy.xmin;
    if (first_xy.ymax - first_xy.ymin != second_xy.ymax - second_xy.ymin)//高度长
        return first_xy.ymax - first_xy.ymin >= second_xy.ymax - second_xy.ymin;
    return first_xy.xmin + first.xadd >= second_xy.xmin + second.xadd;// 内边缘谁远谁放
}

void perior_geometry_put() {
    pgqueue.clear();
    cout << "图层加入加几个图" << endl;
    int n; cin >> n;
    while (n--) {
        perior_geometry temp;
        cout << "加入的几何结构类型为：" << endl;
        cin >> temp.type;
        temp.type--;
        if (temp.type > polygons.size()) {
            cout << "没有该类型的几何结构哦" << endl;
            n++;
            continue;
        }
        cin >> temp.xadd >> temp.yadd;
        pgqueue.push_back(temp);
    }
    cout << "成功" << endl;
    sort(pgqueue.begin(), pgqueue.end(), cmd);
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++)
    {
        cout << "类型" << (*itt).type + 1 << "原点改变量" << '(' << (*itt).xadd << ',' << (*itt).yadd << ')' << endl;
    }
    return;
}

void get_geometry_layer() {
    perior_geometry_put();
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++) {

        Polygon_2 newone = polygons[(*itt).type];
        for (auto it = newone.vertices_begin(); it != newone.vertices_end(); ++it)
        {
            *it = Point_2(it->x() + (*itt).xadd, it->y() + (*itt).yadd);
        }
        if (judge_polygons_collision(newone)) {
            cout << "加入的几何图形与原有几何图形发生碰撞";
            continue;
        }
        else {
            cout << "几何图形生成成功";
            cout << "Polygon Vertices:" << endl;
            for (auto it = newone.vertices_begin(); it != newone.vertices_end(); ++it)
            {
                cout << "(" << it->x() << ", " << it->y() << ")" << endl;
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