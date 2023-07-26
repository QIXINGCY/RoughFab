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
    std::vector<Vector2d1> polygons;  // ��������еĶ���μ���
    double score;                    // ��������ĵ÷�
    std::set<int> typenum;          //��������ж�������
    // ���캯��
    Candidate(const std::vector<Vector2d1>& polys, double sc, const std::set<int>& tn) : polygons(polys), score(sc), typenum(tn) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //����
    }
    bool operator>(const Candidate& other) const {
        return score > other.score; //����
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
    // ���������������ж������������������õ����ռ����Ϊ����
    double maxYScaledArea = (maxY - minY) * (maxX - minX);
    return totalArea / maxYScaledArea;
}

bool Beamsearch::doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2) {//��ײ��⣬�������
    for (const Vector2d1& one_polygon : poly2) {
        if (PL().CGAL_2D_Two_Polygons_Intersection_C(poly1, one_polygon) > 0) {
            return true; // ������ײ
        }
    }
    return false; // δ������ײ
}

Vector2d1 Beamsearch::translatePolygon(const Vector2d1& polygon, double dx, double dy) {
    std::vector<Vector2d> translatedVertices;
    // �������ж��㣬��ÿ���������ƽ�Ʋ���������ӵ��µĶ����б���
    for (auto it = polygon.begin(); it != polygon.end(); ++it) {

        Vector2d translatedPoint((*it).x + dx, (*it).y + dy);
        translatedVertices.push_back(translatedPoint);
    }
    // ʹ���µĶ����б���һ���µ�Vector2d1���󲢷���
    return Vector2d1(translatedVertices.begin(), translatedVertices.end());
}

std::vector<Vector2d1> Beamsearch::beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>, greater<Candidate>> candidates;

    // ��ʼ��һ���յĺ�ѡ�����������
    candidates.push(Candidate({}, 0.0, {}));

    // ����һ�������ȷ���˳�������Ķ���ν���������������˰�������Ӵ�С�����ʾ��
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    //std::sort(sortedPolygons.begin(), sortedPolygons.end(), [](const Vector2d1& poly1, const Vector2d1& poly2) {
    //    // ���ȷ���˳����бȽϣ����԰���������߳��Ƚ�������
    //    return std::abs(poly1.area()) > std::abs(poly2.area());
    //    });

    // ����ÿ�������õ�λ��
    for (int times = 0; times < sortedPolygons.size(); times++) {
        std::priority_queue < Candidate, std::vector<Candidate>, greater<Candidate>> nextCandidates;

        // ����ǰ�ִε�ÿ����ѡ�������
        while (!candidates.empty())
        {
            Candidate candidate = candidates.top();
            candidates.pop();
            int tab = 0;
            // �����ڵ�ǰλ�÷��ò�ͬ�Ķ����
            for (int i = 0; i < sortedPolygons.size(); i++)
            {
                // ���������Ѿ������ڽ�������У�������
                if (candidate.typenum.count(i) != 0)
                {
                    continue;
                }
                if (tab < 3)tab++;
                else break;
                //������η��������������
                Vector2d bomin, bomax, somin, somax;
                Functs::GetBoundingBox(boundingRect, bomin, bomax);
                Functs::GetBoundingBox(sortedPolygons[i], somin, somax);
                double dx = 0.0;
                double dy = bomax.y - somax.y;
                Vector2d1 finalPolygon = translatePolygon(sortedPolygons[i], dx, dy);
                // ʹ�ö��ַ�����ƽ�ƣ�ֱ��������ײ
                Functs::GetBoundingBox(finalPolygon, bomin, bomax);
                double bottom_distance = bomin.y;


                bool judge = 1;
                double pymin = bomin.y;
                while (bottom_distance > 10)
                {
                    pymin -= bottom_distance;
                    if (pymin < 0) {
                        cout << "�����߽���ײ" << endl;
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
                // �����µĺ�ѡ�������
                std::vector<Vector2d1> newPolygons = candidate.polygons;
                std::set<int> temp = candidate.typenum;
                temp.insert(i);
                newPolygons.push_back(finalPolygon);

                // �����µĽ�������ĵ÷�
                double newScore = calculateScore(newPolygons);
                cout << newScore << endl;//�������
                // ���µĽ��������ӵ���ѡ������

                geometry_layer_output(newPolygons);
                nextCandidates.push(Candidate(newPolygons, newScore, temp));
                // ���ֺ�ѡ���еĴ�С�����������
                if (nextCandidates.size() > beamWidth) {
                    cout << "���ӳ���" << nextCandidates.top().score << endl;
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }
    // ��ȡ��ѵĺ�ѡ�������
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

    // �����ѽ������
    std::cout << "��ѽ��������" << std::endl;
    for (const Vector2d1& polygon : bestSolution) {
        // �������ε�����
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

void Beamsearch::get_points_to_polygon() {
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
    else if (choose == 2) {//�����������
        Vector2d1 points;
        std::cout << "�����뼸�������:";
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
        std::cout << "����ģʽ����" << endl;
    }
}

std::vector<Vector2d1> Beamsearch::perior_geometry_put()
{
    std::vector<Vector2d1> ans;
    vector<int> tt;
    std::cout << "ͼ�����Ӽ���ͼ" << endl;
    int n; cin >> n;
    while (n--) {
        std::cout << "����ļ��νṹ����Ϊ��" << endl;
        int type;
        cin >> type;
        if (type > polygons.size()) {
            std::cout << "û�и����͵ļ��νṹŶ" << endl;
            n++;
            continue;
        }
        Vector2d1 temp = polygons[type - 1];
        ans.push_back(temp);
        tt.push_back(type);
    }
    std::cout << "�ɹ�, Ŀǰ�����У�" << endl;
    for (auto pt : tt) {
        cout << "������Ϊ" << pt << " ��Ԫ��" << endl;
    }
    return ans;
}

void Beamsearch::geometry_layer_output(vector<Vector2d1> a) {
    // ����ͼ��ĳߴ�

    // ����һ����ɫ��ͼ��
    cv::Mat rightimage(boxy, boxx / 2, CV_64FC3, cv::Scalar(0, 0, 0));

    // ���ƶ����
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
    // ��ʾͼ��
    cv::imshow("Polygons", symmetric_image);
    cv::waitKey(0);
    return;
}