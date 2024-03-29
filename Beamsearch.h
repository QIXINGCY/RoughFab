#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Partition_traits_2.h>
#include <CGAL/partition_2.h>
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
#include <igl/opengl/glfw/Viewer.h>
#include<io.h>
#include<direct.h>
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
typedef K::FT FT;  // 浮点数类型
typedef K::Segment_2 Segment;  // 二维线段类型

typedef CGAL::Alpha_shape_vertex_base_2<K> Vb;  // Alpha Shape 顶点类型
typedef CGAL::Alpha_shape_face_base_2<K> Fb;    // Alpha Shape 面类型
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;  // 三角剖分数据结构类型
typedef CGAL::Delaunay_triangulation_2<K, Tds> Triangulation_2;  // Delaunay 三角剖分类型
typedef CGAL::Alpha_shape_2<Triangulation_2> Alpha_shape_2;  // Alpha Shape 类型
typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;  // Alpha Shape 边迭代器
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, CGAL::Default, Itag> CDT;
typedef CDT::Point_2 Point;

typedef CGAL::Partition_traits_2<K> Traits;
typedef Traits::Polygon_2 Polygon2;
typedef std::list<Polygon2> Polygon_list;
using namespace std;
using namespace PGL;
using namespace PPGL;

double boxx, boxy;

// 结构体 TriIndex 用于表示三角形的索引
struct TriIndex {
    unsigned value[3];

    TriIndex() {
        for (unsigned i = 0; i < 3; ++i)
            value[i] = -1;
    }

    TriIndex(unsigned i1, unsigned i2, unsigned i3) {
        value[0] = i1;
        value[1] = i2;
        value[2] = i3;
    }

    unsigned& operator[](unsigned i) {
        return value[i];
    }
};

// 使用 TriIndex 的 vector 类型来表示多个三角形的索引
typedef std::vector<TriIndex> TriIndices;

unsigned findVertexIndex(Point pt, std::vector<Point> points) {
    std::vector<Point>::iterator item = std::find(points.begin(), points.end(), pt);
    if (item != points.end()) {
        unsigned pos = std::distance(points.begin(), item);
        return pos;
    }
    return -1;
}

TriIndices GetTriIndices(std::vector<Point> points) {
    // 先进行凹多边形划分为凸多边形集合
    Polygon_list polys;
    CGAL::approx_convex_partition_2(points.begin(), points.end(), std::back_inserter(polys));

    CDT cdt;
    Polygon2 poly;
    unsigned vertIndex;
    TriIndex triIndex;
    TriIndices triIndices;

    for (Polygon_list::iterator it = polys.begin(); it != polys.end(); ++it) {
        // 约束三角剖分
        cdt.clear();
        cdt.insert(it->vertices_begin(), it->vertices_end());

        for (CDT::Finite_faces_iterator it = cdt.finite_faces_begin(); it != cdt.finite_faces_end(); ++it) {
            for (unsigned i = 0; i < 3; ++i) {
                Point point = it->vertex(i)->point();
                vertIndex = findVertexIndex(point, points);
                triIndex[i] = vertIndex;
            }
            triIndices.push_back(triIndex);
        }
    }
    return triIndices;
}

vector<vector<pair<Eigen::MatrixXd, Eigen::MatrixXi>>> Convert_Polygons_to_Matrix(vector<vector<Vector2d1>> process_solutions)
{
    vector<vector<pair<Eigen::MatrixXd, Eigen::MatrixXi>>> need;

    for (auto polygons : process_solutions)
    {
        vector<pair<Eigen::MatrixXd, Eigen::MatrixXi>> way;

        for (auto polygon : polygons)
        {
            Eigen::MatrixXd V(polygon.size(), 2);
            int i = 0;
            vector<Point> a;
            for (auto point : polygon)
            {
                V(i, 0) = point.x;
                V(i, 1) = point.y;
                i++;
                a.push_back(Point(point.x, point.y));
            }
            TriIndices b = GetTriIndices(a);
            Eigen::MatrixXi F(b.size(), 3);
            int j = 0;
            for (auto face : b)
            {
                F(j, 0) = face.value[0];
                F(j, 1) = face.value[1];
                F(j, 2) = face.value[2];
                j++;
            }
            pair<Eigen::MatrixXd, Eigen::MatrixXi> we(V, F);
            way.push_back(we);
        }
        need.push_back(way);
    }

    return need;
}

double pointToPolygonDist(const Point_2& p, const Polygon_2& polygon) {//点到多边形的距离;检验完成，没报错
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
Polygon_2 Convert_Vector2d1_to_Polygon_2(Vector2d1 v2d)
{
    vector<Point_2> pts;
    for (auto itt = v2d.begin(); itt != v2d.end(); itt++)
    {
        Point_2 temp((*itt).x, (*itt).y);
        pts.push_back(temp);
    }
    Polygon_2 target(pts.begin(), pts.end());
    return target;
}

Vector2d1 Convert_Polygon_2_to_Vector2d1(Polygon_2 p2)
{
    Vector2d1 target;
    for (auto itt = p2.begin(); itt != p2.end(); itt++)
    {
        Vector2d temp((*itt).x(), (*itt).y());
        target.push_back(temp);
    }
    return target;
}
struct SegmentComparator {
    bool operator()(const Segment_2& a, const Segment_2& b) const {
        // 实现比较逻辑，返回 true 如果 seg1 应该排在 seg2 之前
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
        // 遍历三角形的三条边.
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
        //cv::Mat image(boxy + 200, boxx + 200, CV_8UC3, cv::Scalar(255, 255, 255));
        for (auto it = (*itt).begin(); it != (*itt).end(); it++)
        {
            Segment_2 edge = (*it);
            pots.push_back(Point_2(edge.source().x(), edge.source().y()));
            /*cv::line(image, cv::Point(edge[0].x() + 100, edge[0].y() + 100),
                cv::Point(edge[1].x() + 100, edge[1].y() + 100), cv::Scalar(0, 0, 0), 1);*/
        }
        Polygon_2 wewant(pots.begin(), pots.end());
        true_ans.push_back(wewant);
        // 显示图像
        /*cv::imshow("Image with Point", image);
        cv::waitKey(0);*/
    }
    return true_ans;
}

class TreeNode {
public:
    int id;
    std::vector<TreeNode*> children;

    TreeNode(int tid) : id(tid) {}
};

class Tree {
public:
    TreeNode* root;

    Tree() : root(nullptr) {}

    void insert(int parentId, int Id) {
        if (root == nullptr) {
            if (parentId == -1) {
                root = new TreeNode(Id);
            }
            return;
        }

        insertRec(root, parentId, Id);
    }

    void insertRec(TreeNode* node, int parentId, int Id) {
        if (node->id == parentId) {
            node->children.push_back(new TreeNode(Id));
        }
        else {
            for (TreeNode* child : node->children) {
                insertRec(child, parentId, Id);
            }
        }
    }

    void printTree(TreeNode* node) {
        if (node == nullptr) {
            return;
        }

        std::cout << node->id << ": ";
        for (TreeNode* child : node->children) {
            std::cout << child->id << " ";
        }
        std::cout << std::endl;

        for (TreeNode* child : node->children) {
            printTree(child);
        }
    }
};

struct Candidate {
    int CandidateId = 0;
    std::vector<Vector2d1> polygons;  // 解决方案中的多边形集合
    double score;                    // 解决方案的得分
    std::set<int> typenum;          //解决方案中多边形序号
    // 构造函数
    Candidate(int id, const std::vector<Vector2d1>& polys, double sc, const std::set<int>& tn) : CandidateId(id), polygons(polys), score(sc), typenum(tn) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //排序
    }
    bool operator>(const Candidate& other) const {
        return score > other.score; //排序
    }
};

class Beamsearch {
public:
    string image_path = "image_outputs";
    Tree gml_tree;
    vector<Vector2d1> polygons;
    vector<vector<Vector2d1>> process_solutions;
    vector<double> score;
    double calculateScore(const std::vector<Vector2d1>& polygons);
    bool doPolygonsCollide2(const Vector2d1& poly1, const vector<Vector2d1>& poly2);
    Vector2d1 translatePolygon(const Vector2d1& polygon, double dx, double dy);
    std::vector<Vector2d1> beamSearch(const std::vector<Vector2d1>& inputPolygons, int beamWidth, const Vector2d1& boundingRect);
    void work();
    void test();
    void get_points_to_polygon();
    std::vector<Vector2d1> perior_geometry_put();
    void geometry_layer_output(vector<Vector2d1> a);
    void geometry_layer_save(vector<Vector2d1> a, int num, double score);
    vector<double> GetScore();
    void Convert_GmlTree_To_GML();
    void generateNodeGML(TreeNode* node, std::ofstream& outfile);
    void generateEdgeGML(TreeNode* node, std::ofstream& outfile);

};
void Beamsearch::generateNodeGML(TreeNode* node, std::ofstream& outfile) {
    if (node == nullptr) {
        return;
    }
    outfile << "   node" << std::endl;
    outfile << "   [" << std::endl;
    outfile << "      id " << node->id << std::endl;
    outfile << "   ]" << std::endl;
    for (TreeNode* child : node->children) {
        generateNodeGML(child, outfile);
    }
}
void Beamsearch::generateEdgeGML(TreeNode* node, std::ofstream& outfile) {
    if (node == nullptr) {
        return;
    }
    for (TreeNode* child : node->children) {
        outfile << "   edge" << std::endl;
        outfile << "   [" << std::endl;
        outfile << "      source " << node->id << std::endl;
        outfile << "      target " << child->id << std::endl;
        outfile << "   ]" << std::endl;
        generateEdgeGML(child, outfile);
    }
}
void Beamsearch::Convert_GmlTree_To_GML() {
    std::ofstream outfile("tree.gml");
    if (outfile.is_open()) {
        outfile << "graph" << std::endl;
        outfile << "[" << std::endl;
        outfile << "   directed 0" << std::endl;

        generateNodeGML(gml_tree.root, outfile);
        generateEdgeGML(gml_tree.root, outfile);

        outfile << "]" << std::endl;
        outfile.close();
    }
    else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }

}
vector<double> Beamsearch::GetScore() {
    return score;
}
double Beamsearch::calculateScore(const std::vector<Vector2d1>& polygons) {
    double areas = 0;
    double score = 0;
    vector<Polygon_2> pys;
    vector<Polygon_2> ans;
    vector<Vector2d1> output;
    for (auto it = polygons.begin(); it != polygons.end(); it++)
    {
        pys.push_back(Convert_Vector2d1_to_Polygon_2(*it));
    }
    vector<Point_2> getit;
    CGAL_2D_Polygon_Dart_Sampling_b(pys, 0.5, getit, 100);
    ans = get_triangulation_net(getit, pys);
    for (auto it = ans.begin(); it != ans.end(); it++) {
        output.push_back(Convert_Polygon_2_to_Vector2d1(*it));
    }
    //geometry_layer_output(output);
    for (auto it = ans.begin(); it != ans.end(); it++)
    {
        if (abs(it->area()) < 500) {
            continue;
        }
        else {
            areas += it->bbox().x_span() * it->bbox().y_span();
            double length = 0;

            for (auto itt = it->edges_begin(); itt != it->edges_end(); itt++)
            {
                length += sqrt(itt->squared_length());
            }
            score += it->bbox().x_span() * it->bbox().y_span() * (it->bbox().x_span() + it->bbox().y_span()) * 2 / (length);
        }
    }
    if (areas == 0)return 0;
    score /= areas;
    return score;
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
    int id = 0;
    std::priority_queue < Candidate, std::vector<Candidate>, less<Candidate>> candidates;
    Candidate root(id++,{}, 0.0, {});
    // 初始化一个空的候选解决方案集合
    candidates.push(root);
    gml_tree.insert(-1, 0);
    // 按照一定的优先放置顺序对输入的多边形进行排序，这里采用了按照面积从大到小排序的示例
    std::vector<Vector2d1> sortedPolygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), beamssort);

    // 处理每个待放置的位置
    for (int times = 0; times < sortedPolygons.size(); times++) {
        std::priority_queue < Candidate, std::vector<Candidate>, less<Candidate>> nextCandidates;

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
                cout << "方案id" << id << ":" << newScore << endl;//输出评分
                // 将新的解决方案添加到候选集合中
                geometry_layer_save(newPolygons,id,newScore);
                score.push_back(newScore);
                process_solutions.push_back(newPolygons);
                gml_tree.insert(candidate.CandidateId, id);
                Candidate son(id++, newPolygons, newScore, temp);
                nextCandidates.push(son);
                // 保持候选队列的大小不超过束宽度
                while (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }
        candidates = nextCandidates;
    }
    // 获取最佳的候选解决方案
    while (candidates.size() > 1) {
        candidates.pop();
    }
    cout << "score" << candidates.top().score << endl;
    return candidates.top().polygons;
}

void create_folder(string a) {
    string folderPath = "./" + a;
    CreateDirectory(folderPath.c_str(), NULL);
    return;
}

void Beamsearch::work() {
    string output_filename = image_path;
    create_folder(output_filename);
    SYSTEMTIME st;
    GetSystemTime(&st);
    string time_path = image_path + "/" + to_string(st.wYear) + "_" + to_string(st.wMonth) + "_" + to_string(st.wDay) + "_" + to_string(st.wHour) + "_" + to_string(st.wMinute) + "_" + to_string(st.wSecond);
    create_folder(time_path);
    this->image_path = "./" + time_path;
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

void Beamsearch::test() {
    get_points_to_polygon();

}

void Beamsearch::get_points_to_polygon() {
    boxx = 700;
    boxy = 700;
    string address = "test.txt";
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

    //    infile.close();
    //std::cout << "请输入处理原件的垂直截面的x，y" << endl;//如问题
    //cin >> boxx >> boxy;
    //std::cout << "如果你想加载已有的几何结构，请输入‘1’；你要创建新的几何结构，请输入‘2’:";
    //int choose;
    //cin >> choose;//choose决定你是要创建还是加载
    //if (choose == 1) {
    //    std::cout << "请输入打开的文件是：";
    //    string address;
    //    cin >> address;
    //    ifstream infile;
    //    infile.open(address);
    //    if (!infile.is_open()) {
    //        std::cout << "文件打开失败" << endl;
    //        return;
    //    }
    //    string line;
    //    while (getline(infile, line)) {//每次从文件读取一行
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
    //else if (choose == 2) {//和上面很相像
    //    Vector2d1 points;
    //    std::cout << "请输入几何体点数:";
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
    //    std::cout << "输入模式错误！" << endl;
    //}
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
void Beamsearch::geometry_layer_save(vector<Vector2d1> a,int num,double score) {
    // 计算图像的尺寸
    string path = this->image_path;
    path = path + "/节点" + to_string(num) +"评分：" + to_string(score) + "\.jpg";
    cout << path << endl;
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
    cv::imwrite(path, symmetric_image);
    cv::waitKey(0);
    return;
}