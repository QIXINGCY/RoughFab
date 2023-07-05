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

vector<Polygon_2> polygons;//���νṹ
vector<Polygon_2> geometry_layer;//����ͼ��
vector<perior_geometry> pgqueue;
vector<polygons_xy_information> pxyinformation;

void get_more_geometry();
void get_points_to_polygon();//��ü��νṹ
bool judge_polygons_collision(Polygon_2 polygon_target);//��ײ�ж�
void get_geometry_layer();//���νṹ����ͼ�㲢�Ų�
void perior_geometry_put();
polygons_xy_information find_type_xy(int type);
void geometry_layer_output();//ͼ�����
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
    cout << "�����봦��ԭ���Ĵ�ֱ�����x��y" << endl;
    cin >> boxx >> boxy;
    cout << "Ҫ���뼸�����νṹ����" << endl;
    int n;
    cin >> n;
    while (n--)
    {
        get_points_to_polygon();
    }
}
void get_points_to_polygon() {
    cout << "�������������еļ��νṹ�������롮1������Ҫ�����µļ��νṹ�������롮2��:";
    int choose;
    cin >> choose;
    if (choose == 1) {
        cout << "������򿪵��ļ��ǣ�";
        string address;
        cin >> address;
        ifstream infile;
        infile.open(address);
        if (!infile.is_open()) {
            cout << "�ļ���ʧ��" << endl;
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

        cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
        cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        infile.close();
    }
    else if (choose == 2) {
        vector<Point_2> points;
        cout << "�����뼸�������:";
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

        cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
        cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        pxyinformation.push_back(xy);


    }
    else {
        cout << "����ģʽ����" << endl;
    }
}



bool judge_polygons_collision(Polygon_2 polygon_target) {
    for (const Polygon_2& polygon : geometry_layer) {
        if (CGAL::do_intersect(polygon, polygon_target)) {
            return true; // ������ײ
        }
    }
    return false; // δ������ײ
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
    if (first_xy.xmax + first.xadd != second_xy.xmax + second.xadd)//���Ե˭Զ˭��
        return first_xy.xmax + first.xadd >= second_xy.xmax + second.xadd;
    if (first_xy.xmax - first_xy.xmin != second_xy.xmax - second_xy.xmin)//���ȳ�
        return first_xy.xmax - first_xy.xmin >= second_xy.xmax - second_xy.xmin;
    if (first_xy.ymax - first_xy.ymin != second_xy.ymax - second_xy.ymin)//�߶ȳ�
        return first_xy.ymax - first_xy.ymin >= second_xy.ymax - second_xy.ymin;
    return first_xy.xmin + first.xadd >= second_xy.xmin + second.xadd;// �ڱ�Ե˭Զ˭��
}

void perior_geometry_put() {
    pgqueue.clear();
    cout << "ͼ�����Ӽ���ͼ" << endl;
    int n; cin >> n;
    while (n--) {
        perior_geometry temp;
        cout << "����ļ��νṹ����Ϊ��" << endl;
        cin >> temp.type;
        temp.type--;
        if (temp.type > polygons.size()) {
            cout << "û�и����͵ļ��νṹŶ" << endl;
            n++;
            continue;
        }
        cin >> temp.xadd >> temp.yadd;
        pgqueue.push_back(temp);
    }
    cout << "�ɹ�" << endl;
    sort(pgqueue.begin(), pgqueue.end(), cmd);
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++)
    {
        cout << "����" << (*itt).type + 1 << "ԭ��ı���" << '(' << (*itt).xadd << ',' << (*itt).yadd << ')' << endl;
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
            cout << "����ļ���ͼ����ԭ�м���ͼ�η�����ײ";
            continue;
        }
        else {
            cout << "����ͼ�����ɳɹ�";
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
    // ����ͼ��ĳߴ�
    int width = 1000;
    int height = 800;

    // ����һ����ɫ��ͼ��
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // ���ƶ����
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
    // ��ʾͼ��
    cv::imshow("Polygons", image);
    cv::waitKey(0);
    return;
}