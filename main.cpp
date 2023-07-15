#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include<bits/stdc++.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <opencv2/opencv.hpp>
#include"build/search_core/beam_search.h"

using namespace std;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;//��
typedef CGAL::Polygon_2<K> Polygon_2;

struct perior_geometry {//�������ͣ��������࣬����x��yƫ����
    int type;
    int yadd;
};
struct polygons_xy_information {//��Χ����Ϣ����򵥵İ�Χ�У�x��y�����Сֵ
    int type;
    int xmin, xmax, ymin, ymax;
};
int boxx, boxy;//ȫ�ֱ����������͵�x��yֵ

vector<Polygon_2> polygons;//���νṹ
vector<Polygon_2> geometry_layer;//����ͼ��
vector<perior_geometry> pgqueue;
vector<polygons_xy_information> pxyinformation;//��Χ�ж���

void get_more_geometry();//ѭ�������������
void get_points_to_polygon();//��ü��νṹ
bool judge_polygons_collision(Polygon_2 polygon_target);//��ײ�ж�
void get_geometry_layer();//���νṹ����ͼ�㲢�Ų�
void perior_geometry_put();
polygons_xy_information find_type_xy(int type);
void geometry_layer_output();//ͼ�����
bool cmd(perior_geometry first, perior_geometry second);

// �ṹ�����ڱ�ʾ��ѡ�������
struct Candidate {
    std::vector<Polygon_2> polygons;  // ��������еĶ���μ���
    double score;                    // ��������ĵ÷�
    // ���캯��
    Candidate(const std::vector<Polygon_2>& polys, double sc) : polygons(polys), score(sc) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; // �Ӵ�С����
    }
};

// �����ѡ��������ĵ÷�
double calculateScore(const std::vector<Polygon_2>& polygons) {
    // ���ݾ������ⶨ�����ֺ���
    // ���磬���Լ������θ��ǵ������
    double totalArea = 0.0;
    for (const Polygon_2& polygon : polygons) {
        totalArea += polygon.area();
    }
    return totalArea;
}

// �������������Ƿ�����ײ���ص���
bool doPolygonsCollide(const Polygon_2& poly1, const Polygon_2& poly2) {
    if (CGAL::do_intersect(poly1, poly2)) {
        return true; // ������ײ
    }
    return false;
}

// ƽ��Polygon_2�е����е�����
void translatePolygon(Polygon_2& polygon, double dx, double dy) {
    // �������ж��㣬��ÿ���������ƽ�Ʋ���

}

// ִ���������㷨
std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>> candidates;
    // ��ʼ��һ���ս������
    candidates.push(Candidate({}, 0.0));
    // ����ÿ����������
    for (const Polygon_2& polygon : inputPolygons)
    {
        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;

        // ����ǰ�ִε�ÿ����ѡ�������
        while (!candidates.empty())
        {
            Candidate candidate = candidates.top();
            candidates.pop();

            // �ڲ�ͬλ�ò��뵱ǰ����Σ������µĺ�ѡ�������
            for (int i = 0; i <= candidate.polygons.size(); ++i)
            {
                std::vector<Polygon_2> newPolygons = candidate.polygons;
                newPolygons.insert(newPolygons.begin() + i, polygon);
                // ����������еĶ�����Ƿ�����ײ
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
                    // ������y�����ƶ���������Ż�λ��
                    double currentScore = calculateScore(newPolygons);
                    double newY = newPolygons[i].bbox().ymax();  // ��ǰλ�õ�y����
                    double stepSize = 0.01;  // ����
                    double bestY = newY;
                    for (int k = 0; k < 10; ++k)
                    {
                        newY += stepSize; // ������y���ϼ��ϲ���

                        double newScore = calculateScore(newPolygons);
                        // ����÷ָ��ã�������λ��
                        if (newScore > currentScore)
                        {
                            bestY = newY;
                            currentScore = newScore;
                        }
                    }
                }
                else
                {
                    // ������y�����ƶ���������Ż�λ��
                    double currentScore = calculateScore(newPolygons);
                    double newY = newPolygons[i].bbox().ymax();  // ��ǰλ�õ�y����
                    double stepSize = 0.01;  // ����
                    double bestY = newY;
                    for (int k = 0; k < 10; ++k)
                    {
                        newY -= stepSize; // ������y���ϼ��ٲ���

                        double newScore = calculateScore(newPolygons);
                        // ����÷ָ��ã�������λ��
                        if (newScore > currentScore)
                        {
                            bestY = newY;
                            currentScore = newScore;
                        }
                    }
                }
                // ���ֺ�ѡ���еĴ�С�����������
                if (nextCandidates.size() > beamWidth)
                {
                    nextCandidates.pop();
                }
            }
        }
        candidates = nextCandidates;
    }
    // ��ȡ��ѵĺ�ѡ�������
    std::vector<Polygon_2> bestSolution = candidates.top().polygons;
    // ����ѽ���������б�Ҫ�ĺ���
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

    // �����ѽ������
    std::cout << "��ѽ��������" << std::endl;
    for (const Polygon_2& polygon : bestSolution) {
        // �������ε�����
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
    std::cout << "�����봦��ԭ���Ĵ�ֱ�����x��y" << endl;//������
    cin >> boxx >> boxy;
    std::cout << "Ҫ���뼸�����νṹ���ͻ�����Ҫ���뼸���ṹ�ļ�" << endl;
    int n;
    cin >> n;
    while (n--)
    {
        get_points_to_polygon();
    }
}
void get_points_to_polygon() {
    std::cout << "�������������еļ��νṹ�������롮1������Ҫ�����µļ��νṹ�������롮2��:";
    int choose;
    cin >> choose;//choose��������Ҫ�������Ǽ���
    if (choose == 1) {
        std::cout << "������򿪵��ļ��ǣ�";
        string address;
        cin >> address;
        ifstream infile;
        infile.open(address);
        if (!infile.is_open()) {
            std::cout << "�ļ���ʧ��" << endl;
            return;
        }
        string line;
        while (getline(infile, line)) {//ÿ�δ��ļ���ȡһ��
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
            iss >> xy.xmax >> xy.xmin >> xy.ymax >> xy.ymin;//x,y���ֵ��Сֵ
            Polygon_2 polygontemp(points.begin(), points.end());
            xy.type = polygons.size() + 1;
            pxyinformation.push_back(xy);
            polygons.push_back(polygontemp);

            std::cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
            std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
            std::cout << "Polygon Vertices:" << endl;
            for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
                std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;
        }

        infile.close();
    }
    else if (choose == 2) {//�����������
        vector<Point_2> points;
        std::cout << "�����뼸�������:";
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

        std::cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
        std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        std::cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        pxyinformation.push_back(xy);


    }
    else {
        std::cout << "����ģʽ����" << endl;
    }
}


bool judge_polygons_collision(Polygon_2 polygon_target) {//��ײ��⣬�������
    for (const Polygon_2& polygon : geometry_layer) {
        if (CGAL::do_intersect(polygon, polygon_target)) {
            return true; // ������ײ
        }
    }
    return false; // δ������ײ
}

polygons_xy_information find_type_xy(int type)//�ж��Ƿ�������ͼ�����
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
    if (first_xy.xmax != second_xy.xmax)//���Ե˭Զ˭��
        return first_xy.xmax >= second_xy.xmax;
    if (first_xy.xmax - first_xy.xmin != second_xy.xmax - second_xy.xmin)//���ȳ�
        return first_xy.xmax - first_xy.xmin >= second_xy.xmax - second_xy.xmin;
    if (first_xy.ymax - first_xy.ymin != second_xy.ymax - second_xy.ymin)//�߶ȳ�
        return first_xy.ymax - first_xy.ymin >= second_xy.ymax - second_xy.ymin;
    return first_xy.xmin >= second_xy.xmin;// �ڱ�Ե˭Զ˭��
}

void perior_geometry_put() {
    pgqueue.clear();
    std::cout << "ͼ�����Ӽ���ͼ" << endl;
    int n; cin >> n;
    while (n--) {
        perior_geometry temp;
        std::cout << "����ļ��νṹ����Ϊ��" << endl;
        cin >> temp.type;
        temp.type--;
        if (temp.type > polygons.size()) {
            std::cout << "û�и����͵ļ��νṹŶ" << endl;
            n++;
            continue;
        }
        cin >> temp.yadd;
        pgqueue.push_back(temp);
    }
    std::cout << "�ɹ�" << endl;
    sort(pgqueue.begin(), pgqueue.end(), cmd);
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++)
    {
        std::cout << "����" << (*itt).type + 1 << "ԭ��ı���" << (*itt).yadd << endl;
    }
    return;
}

void get_geometry_layer() {
    perior_geometry_put();
    for (auto itt = pgqueue.begin(); itt != pgqueue.end(); itt++) {

        Polygon_2 newone = polygons[(*itt).type];//׼�������
        for (auto it = newone.vertices_begin(); it != newone.vertices_end(); ++it)//��ѭ��
        {
            *it = Point_2(it->x(), it->y() + (*itt).yadd);//λ��
        }
        if (judge_polygons_collision(newone)) {
            std::cout << "����ļ���ͼ����ԭ�м���ͼ�η�����ײ";
            continue;
        }
        else {
            std::cout << "����ͼ�����ɳɹ�";
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






