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
#include <CGAL/convex_hull_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <CGAL/Alpha_shape_vertex_base_2.h>
#include <CGAL/Alpha_shape_face_base_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef CGAL::Polygon_2<K> Polygon_2;
typedef K::Segment_2 Segment_2;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;
typedef K::FT FT;  // ����������
typedef K::Segment_2 Segment;  // ��ά�߶�����

using namespace std;
using namespace PGL;
using namespace PPGL;

double boxx, boxy;




double pointToPolygonDist(const Point_2& p, const Polygon_2& polygon) {//�㵽����εľ���;������ɣ�û����
    int count = 0;
    double minDist = INFINITY;

    for (auto e = polygon.edges_begin(); e != polygon.edges_end(); ++e) {
        const Point_2& a = e->source();
        const Point_2& b = e->target();

        // �жϵ� p ���߶� ab �Ƿ���ͬһˮƽ���ϣ����� p �� ab �����
        if ((a.y() > p.y() != b.y() > p.y()) &&
            (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x())) {
            count++;
        }

        // ����� p ���߶� ab ����̾��룬��������С����
        Segment_2 s(a, b);
        minDist = std::min(minDist, squared_distance(p, s));
    }

    if (count % 2 == 0) {
        return std::sqrt(abs(minDist));

    }
    return -std::sqrt(abs(minDist));
}

void CGAL_2D_Polygon_Dart_Sampling_b(const vector<Polygon_2>& py, const double& d, vector<Point_2>& sampling_points, const int& total_iter)
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
            Polygon_2 poly = py[num];
            distance = min(distance, pointToPolygonDist(point, poly));

        }
        for (int num = 0; num < insert_points.size(); num++) {
            distance = min(distance, squared_distance(point, insert_points[num]));

        }
        if (distance > 20)
        {
            insert_points.push_back(Point_2(x, y));
            run = 0;
        }
    }
    sampling_points = insert_points;
}

void CGAL_2D_Polygon_Dart_Sampling_a(const vector<Polygon_2>& py, const double& d, vector<Point_2>& sampling_points)
{
    Functs::MAssert(d > 0 && d < 1.0, "CGAL_2D_Polygon_Dart_Sampling if (!(d > 0 && d < 1.0))");

    double xmin = 0;
    double ymin = 0;
    double xmax = boxx;
    double ymax = boxy;
    double minimal_d = 15;
    int total_iter = 1 / d;
    double dx = xmax / total_iter;
    double dy = ymax / total_iter;
    vector<Point_2> insert_points;
    for (int i = 0; i < total_iter; i++)
    {
        double x = xmin + dx * i;
        for (int j = 0; j < total_iter; j++) {

            double y = ymin + dy * j;
            Point_2 point(x, y);
            double distance = CGAL_IA_MAX_DOUBLE;
            for (int num = 0; num < py.size(); num++) {
                Polygon_2 poly = py[num];
                distance = min(distance, pointToPolygonDist(point, poly));

            }
            if (distance > minimal_d)
            {
                insert_points.push_back(Point_2(x, y));
            }
        }
    }
    sampling_points = insert_points;
}

void findPolygons(vector<Segment_2> edges, vector<vector<Segment_2>>& polys) {
    while (!edges.empty()) {
        vector<Segment_2> poly;
        Segment_2 currentEdge = edges[0];
        poly.push_back(currentEdge);
        edges.erase(edges.begin());
        while (true) {
            Segment_2 nextEdge;
            bool found = false;
            for (auto it = edges.begin(); it != edges.end(); it++) {
                if (it->source().x() == currentEdge.target().x() && it->source().y() == currentEdge.target().y())
                {
                    nextEdge = *it;
                    edges.erase(it);
                    found = true;
                    break;
                }
                else if (it->target().x() == currentEdge.target().x() && it->target().y() == currentEdge.target().y())
                {
                    Segment_2 wewant(Point_2(it->target().x(), it->target().y()), Point_2(it->source().x(), it->source().y()));
                    nextEdge = wewant;
                    edges.erase(it);
                    found = true;
                    break;
                }
            }
            if (found) {
                poly.push_back(nextEdge);
                currentEdge = nextEdge;
            }
            else {
                break;
            }
        }
        polys.push_back(poly);
    }
}

struct SegmentComparator {
    bool operator()(const Segment_2& a, const Segment_2& b) const {
        // ʵ�ֱȽ��߼������� true ��� seg1 Ӧ������ seg2 ֮ǰ
        if (a.source().x() != b.source().x()) {
            return a.source().x() < b.source().x();
        }
        else if (a.source().y() != b.source().y()) {
            return a.source().y() < b.source().y();
        }
        else if (a.target().x() != b.target().x()) {
            return a.target().x() < b.target().x();
        }
        else {
            return a.target().y() < b.target().y();
        }

    }
};
bool comPoints(const Point_2& p1, const Point_2& p2) {
    if (p1.x() != p2.x()) {
        return p1.x() < p2.x();
    }
    return p1.y() < p2.y();
}

vector<Polygon_2> get_triangulation_net(vector<Point_2> point_set, vector<Polygon_2> py)
{
    Delaunay triangulation;
    triangulation.insert(point_set.begin(), point_set.end());
    double maxlength = 400;
    vector<Segment_2> one_face_edges;
    map<Segment_2, int, SegmentComparator> edge_map;
    for (Delaunay::Finite_faces_iterator fit = triangulation.finite_faces_begin(); fit != triangulation.finite_faces_end(); ++fit) {
        // ���������ε�������.
        bool tab = false;
        Delaunay::Vertex_handle v1 = fit->vertex(0);
        Delaunay::Vertex_handle v2 = fit->vertex(1);
        Delaunay::Vertex_handle v3 = fit->vertex(2);
        Point_2 wt1 = v1->point();
        Point_2 wt2 = v2->point();
        Point_2 wt3 = v3->point();
        Segment_2 edge1(wt2, wt1);
        Segment_2 edge2(wt3, wt1);
        Segment_2 edge3(wt3, wt2);
        if (comPoints(wt1, wt2)) {
            Segment_2 edge(wt1, wt2);
            edge1 = edge;
        }
        else {
            Segment_2 edge(wt2, wt1);
            edge1 = edge;
        }

        if (comPoints(wt2, wt3)) {
            Segment_2 edge(wt2, wt3);
            edge2 = edge;
        }
        else {
            Segment_2 edge(wt3, wt2);
            edge2 = edge;
        }

        if (comPoints(wt1, wt3)) {
            Segment_2 edge(wt1, wt3);
            edge3 = edge;
        }
        else {
            Segment_2 edge(wt3, wt1);
            edge3 = edge;
        }
        double edge1_length = CGAL::to_double(edge1.squared_length());
        double edge2_length = CGAL::to_double(edge2.squared_length());
        double edge3_length = CGAL::to_double(edge3.squared_length());
        double maxedge = max(edge1_length, max(edge2_length, edge3_length));
        if (maxedge < maxlength) {
            edge_map[edge1]++;
            edge_map[edge2]++;
            edge_map[edge3]++;
        }
    }
    for (auto it = edge_map.begin(); it != edge_map.end(); it++) {
        if (it->second != 2) {
            one_face_edges.push_back(it->first);
        }
    }
    vector<vector<Segment_2>> ans;
    findPolygons(one_face_edges, ans);
    vector<Polygon_2> true_ans;
    for (auto itt = ans.begin(); itt != ans.end(); itt++)
    {
        vector<Point_2> pots;
        cv::Mat image(boxy + 200, boxx + 200, CV_8UC3, cv::Scalar(255, 255, 255));
        for (auto it = (*itt).begin(); it != (*itt).end(); it++)
        {
            Segment_2 edge = (*it);
            pots.push_back(Point_2(edge.source().x(), edge.source().y()));
            cv::line(image, cv::Point(edge[0].x() + 100, edge[0].y() + 100),
                cv::Point(edge[1].x() + 100, edge[1].y() + 100), cv::Scalar(0, 0, 0), 1);
        }
        Polygon_2 wewant(pots.begin(), pots.end());
        true_ans.push_back(wewant);
        // ��ʾͼ��
        cv::imshow("Image with Point", image);
        cv::waitKey(0);
    }
    return true_ans;
}

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
    double calculateScore(const std::vector<Vector2d1>& polygons);
    bool doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2);
    Vector2d1 translatePolygon(const Vector2d1& polygon, double dx, double dy);
    std::vector<Vector2d1> beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect);
    void work();
    void test();
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

    // ��ʼ��һ���յĺ�ѡ�����������
    candidates.push(Candidate({}, 0.0, {}));

    // ����һ�������ȷ���˳�������Ķ���ν���������������˰�������Ӵ�С�����ʾ��
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), beamssort);

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
                if (doPolygonsCollide2(finalPolygon, candidate.polygons) || somax.y > boxy) {
                    tab--;
                    continue;
                }
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
                while (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }
    // ��ȡ��ѵĺ�ѡ�������
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

    // �����ѽ������
    std::cout << "��ѽ��������" << std::endl;
    for (const Vector2d1& polygon : bestSolution) {
        // �������ε�����
        for (const Vector2d& point : polygon) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
    }
    if (bestSolution.empty())cout << "�޷����ɽ��������" << endl;
    else geometry_layer_output(bestSolution);
}

void Beamsearch::test() {
    get_points_to_polygon();
    vector<Polygon_2> pys;
    vector<Polygon_2> nulls;
    for (auto it = polygons.begin(); it != polygons.end(); it++)
    {
        std::vector<Point_2> points;
        for (auto itt = (*it).begin(); itt != (*it).end(); itt++)
        {
            double x, y;
            x = itt->x;
            y = itt->y;
            points.push_back(Point_2(x, y));
        }
        Polygon_2 plg(points.begin(), points.end());
        pys.push_back(plg);
    }
    vector<Point_2> getit;
    CGAL_2D_Polygon_Dart_Sampling_b(pys, 0.5, getit, 100);
    //FT alpha = 0.001* sqrt(boxx * boxx + boxy * boxy);
    //vector<Point_2> ans = computeAlphaShape(getit, 75);
    //cv::Mat image(boxy, boxx , CV_64FC3, cv::Scalar(0, 0, 0));
    //cv::Scalar color(0, 0, 255);
    get_triangulation_net(getit, pys);

}

void Beamsearch::get_points_to_polygon() {
    boxx = 800;
    boxy = 800;
    string address = "test.txt";
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
    }

    //    infile.close();
    //std::cout << "�����봦��ԭ���Ĵ�ֱ�����x��y" << endl;//������
    //cin >> boxx >> boxy;
    //std::cout << "�������������еļ��νṹ�������롮1������Ҫ�����µļ��νṹ�������롮2��:";
    //int choose;
    //cin >> choose;//choose��������Ҫ�������Ǽ���
    //if (choose == 1) {
    //    std::cout << "������򿪵��ļ��ǣ�";
    //    string address;
    //    cin >> address;
    //    ifstream infile;
    //    infile.open(address);
    //    if (!infile.is_open()) {
    //        std::cout << "�ļ���ʧ��" << endl;
    //        return;
    //    }
    //    string line;
    //    while (getline(infile, line)) {//ÿ�δ��ļ���ȡһ��
    //        istringstream iss(line);
    //        Vector2d1 points;
    //        int n;
    //        iss >> n;
    //        double x, y;
    //        for (int i = 0; i < n; i++)
    //        {
    //            iss >> x >> y;
    //            points.push_back(Vector2d(x, y));
    //        }
    //        if (PL().CGAL_2D_Polygon_Is_Clockwise_Oriented_C(points))
    //        {
    //            std::reverse(points.begin(), points.end());
    //        }
    //        polygons.push_back(points);
    //    }

    //    infile.close();
    //}
    //else if (choose == 2) {//�����������
    //    Vector2d1 points;
    //    std::cout << "�����뼸�������:";
    //    int n;
    //    cin >> n;
    //    double x, y;
    //    for (int i = 0; i < n; i++)
    //    {
    //        cin >> x >> y;
    //        points.push_back(Vector2d(x, y));
    //    }
    //    if (PL().CGAL_2D_Polygon_Is_Clockwise_Oriented_C(points))
    //    {
    //        std::reverse(points.begin(), points.end());
    //    }
    //    std::cout << "Polygon Vertices:" << endl;
    //    for (auto it = points.begin(); it != points.end(); ++it)
    //        std::cout << "(" << (*it).x << ", " << (*it).y << ")" << endl;

    //    polygons.push_back(points);
    //}
    //else {
    //    std::cout << "����ģʽ����" << endl;
    //}
}

std::vector<Vector2d1> Beamsearch::perior_geometry_put()
{
    std::vector<Vector2d1> ans = polygons;
    std::cout << "ͼ����������Ƿ��ظ�" << endl;
    bool ques;
    cin >> ques;
    if (ques) {
        std::cout << "�ظ����м���" << endl;
        int n; cin >> n;
        while (n--) {
            int type;
            cin >> type;
            if (type > polygons.size()) {
                std::cout << "û�и����͵ļ��νṹŶ������������" << endl;
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