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
			std::cout << "�ļ���ʧ��" << endl;
			return;
		}

		string line;

		while (getline(infile, line))
		{//ÿ�δ��ļ���ȡһ��
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


struct perior_geometry {//�������ͣ��������࣬����x��yƫ����
    int type;
    int yadd;
};
double boxx, boxy;//ȫ�ֱ����������͵�x��yֵ

Vector2d1 a = { Vector2d(0,0), Vector2d(1,0), Vector2d(1,1)};
Vector2d2 b = {a,a,a};

vector<Polygon_2> polygons;//���νṹ
vector<Polygon_2> geometry_layer;//����ͼ��
vector<perior_geometry> pgqueue;

void get_more_geometry();//ѭ�������������
void get_points_to_polygon();//��ü��νṹ
bool judge_polygons_collision(Polygon_2 polygon_target);//��ײ�ж�
void get_geometry_layer();//���νṹ����ͼ�㲢�Ų�
void perior_geometry_put();
void geometry_layer_output(vector<Polygon_2> a);//ͼ�����
bool cmd(perior_geometry first, perior_geometry second);

// �ṹ�����ڱ�ʾ��ѡ�������
struct Candidate {
    std::vector<Polygon_2> polygons;  // ��������еĶ���μ���
    double score;                    // ��������ĵ÷�
    std::set<int> typenum;          //��������ж�������
    // ���캯��
    Candidate(const std::vector<Polygon_2>& polys, double sc, const std::set<int>& tn) : polygons(polys), score(sc), typenum(tn) {}
    bool operator<(const Candidate& other) const {
        return score < other.score; //����
    }
};

// �����ѡ��������ĵ÷�
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
    // ���������������ж������������������õ����ռ����Ϊ����
    double maxYScaledArea = (maxY - minY) * (maxX - minX);
    return totalArea / maxYScaledArea;
}

// �������������Ƿ�����ײ���ص���
bool doPolygonsCollide(const Polygon_2& poly1, const Polygon_2& poly2) {
    return CGAL::do_intersect(poly1, poly2);
}
// ���һ������κ�һ����������Ƿ�����ײ���ص���
bool doPolygonsCollide2(const Polygon_2& poly1, const vector<Polygon_2>& poly2) {//��ײ��⣬�������
    for (const Polygon_2& one_polygon : poly2) {
        if (CGAL::do_intersect(one_polygon, poly1)) {
            return true; // ������ײ
        }
    }
    return false; // δ������ײ
}

// ƽ��Polygon_2�е����е����겢�����µĶ����
Polygon_2 translatePolygon(const Polygon_2& polygon, double dx, double dy) {
    std::vector<Point_2> translatedVertices;
    // �������ж��㣬��ÿ���������ƽ�Ʋ���������ӵ��µĶ����б���
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        Point_2 translatedPoint(it->x() + dx, it->y() + dy);
        translatedVertices.push_back(translatedPoint);
    }
    // ʹ���µĶ����б���һ���µ�Polygon_2���󲢷���
    return Polygon_2(translatedVertices.begin(), translatedVertices.end());
}

// ִ���������㷨���ڲ�ͬλ�÷��ö����
std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
{
    std::priority_queue < Candidate, std::vector<Candidate>> candidates;

    // ��ʼ��һ���յĺ�ѡ�����������
    candidates.push(Candidate({}, 0.0, {}));

    // ����һ�������ȷ���˳�������Ķ���ν���������������˰�������Ӵ�С�����ʾ��
    std::vector<Polygon_2> sortedPolygons = inputPolygons;
    std::sort(sortedPolygons.begin(), sortedPolygons.end(), [](const Polygon_2& poly1, const Polygon_2& poly2) {
        // ���ȷ���˳����бȽϣ����԰���������߳��Ƚ�������
        return std::abs(poly1.area()) > std::abs(poly2.area());
        });

    // ����ÿ�������õ�λ��
    for (int times = 0; times < inputPolygons.size(); times++) {
        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;

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
                if (tab < 1)tab++;
                else break;
                //�Ѿ����õĶ�������maxY
                /*double maxY = 0.0;
                for (const Polygon_2& poly : candidate.polygons) {
                    for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it) {
                        maxY = std::max(maxY, it->y());
                    }
                }*/

                //������η��������������
                double dx = 0.0;
                double dy = boundingRect.bbox().ymax() - sortedPolygons[i].bbox().ymax();
                Polygon_2 finalPolygon = translatePolygon(sortedPolygons[i], dx, dy);
                // ʹ�ö��ַ�����ƽ�ƣ�ֱ��������ײ
                double bottom_distance = finalPolygon.bbox().ymin();


                bool judge = 1;
                double pymin = finalPolygon.bbox().ymin();
                while (bottom_distance > 1)
                {
                    pymin -= bottom_distance;
                    if (pymin < 0) {
                        cout << "�����߽���ײ" << endl;
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
                //    cout << nnn << "��" << "     " << down << "    " << up <<endl;
                //    double mid = (up + down) / 2.0;
                //    
                //    cout << "���ڴ�����" << i << endl;
                //    Polygon_2 translatedPolygon = translatePolygon(placedPolygon, 0.0, -mid);
                //    for (const int si : candidate.typenum) {
                //        cout << "����" << si << "   ";
                //    }
                //    cout << endl;
                //    bool judge = doPolygonsCollide2(translatedPolygon, candidate.polygons);
                //    if (judge) 
                //    {
                //        down = mid + 1e-9; // ��ײ���������������±߽�
                //        cout << "������ײ" << endl;
                //    }
                //    else 
                //    {
                //        up = mid - 1e-9; // ����ײ�����������ϱ߽�
                //        cout << "û������ײ" << endl;
                //    }
                //}
                //// ���˵����һ������ײ��λ��
                //double finalY = (up + down) / 2.0;
                //


                // �����µĺ�ѡ�������
                std::vector<Polygon_2> newPolygons = candidate.polygons;
                std::set<int> temp = candidate.typenum;
                temp.insert(i);
                newPolygons.push_back(finalPolygon);

                // �����µĽ�������ĵ÷�
                double newScore = calculateScore(newPolygons);
                cout << newScore << endl;//�������
                // ���µĽ��������ӵ���ѡ������
                nextCandidates.push(Candidate(newPolygons, newScore, temp));
                // ���ֺ�ѡ���еĴ�С�����������
                if (nextCandidates.size() > beamWidth) {
                    nextCandidates.pop();
                }
            }
        }

        candidates = nextCandidates;
    }

    // ��ȡ��ѵĺ�ѡ�������
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

    // �����ѽ������
    std::cout << "��ѽ��������" << std::endl;
    for (const Polygon_2& polygon : bestSolution) {
        // �������ε�����
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
            double x, y;
            for (int i = 0; i < n; i++)
            {
                iss >> x >> y;
                points.push_back(Point_2(x, y));
            }
            //polygons_xy_information xy;
            //iss >> xy.xmax >> xy.xmin >> xy.ymax >> xy.ymin;//x,y���ֵ��Сֵ
            Polygon_2 polygontemp(points.begin(), points.end());
            //xy.type = polygons.size() + 1;
            //pxyinformation.push_back(xy);
            polygons.push_back(polygontemp);

            //std::cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
            //std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
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

        //std::cout << "����ͼ�����ɳɹ�,����Ϊ" << xy.type << endl;
        //std::cout << " xmax= " << xy.xmax << " xmin= " << xy.xmin << " ymax= " << xy.ymax << " ymin= " << xy.ymin << endl;
        std::cout << "Polygon Vertices:" << endl;
        for (auto it = polygontemp.vertices_begin(); it != polygontemp.vertices_end(); ++it)
            std::cout << "(" << it->x() << ", " << it->y() << ")" << endl;

        polygons.push_back(polygontemp);
        //pxyinformation.push_back(xy);


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

void geometry_layer_output(vector<Polygon_2> a) {
    // ����ͼ��ĳߴ�

    // ����һ����ɫ��ͼ��
    cv::Mat image(boxy + 10, 2 * boxx + 10, CV_64FC3, cv::Scalar(0, 0, 0));

    // ���ƶ����
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
    // ��ʾͼ��
    cv::imshow("Polygons", image);
    cv::waitKey(0);
    return;
}





//// ִ���������㷨
//std::vector<Polygon_2> beamSearch(const std::vector<Polygon_2>& inputPolygons, int beamWidth, const Polygon_2& boundingRect)
//{
//    std::priority_queue < Candidate, std::vector<Candidate>> candidates;
//    // ��ʼ��һ���ս������
//    candidates.push(Candidate({}, 0.0));
//    // ����ÿ����������
//    for (const Polygon_2& polygon : inputPolygons)
//    {
//        std::priority_queue < Candidate, std::vector<Candidate>> nextCandidates;
//
//        // ����ǰ�ִε�ÿ����ѡ�������
//        while (!candidates.empty())
//        {
//            Candidate candidate = candidates.top();
//            candidates.pop();
//
//            // �ڲ�ͬλ�ò��뵱ǰ����Σ������µĺ�ѡ�������
//            for (int i = 0; i <= candidate.polygons.size(); ++i)
//            {
//                std::vector<Polygon_2> newPolygons = candidate.polygons;
//                newPolygons.insert(newPolygons.begin() + i, polygon);
//                // ����������еĶ�����Ƿ�����ײ
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
//                    // ������y�����ƶ���������Ż�λ��
//                    double currentScore = calculateScore(newPolygons);
//                    double newY = newPolygons[i].bbox().ymax();  // ��ǰλ�õ�y����
//                    double stepSize = 0.01;  // ����
//                    double bestY = newY;
//                    for (int k = 0; k < 10; ++k)
//                    {
//                        newY += stepSize; // ������y���ϼ��ϲ���
//
//                        double newScore = calculateScore(newPolygons);
//                        // ����÷ָ��ã�������λ��
//                        if (newScore > currentScore)
//                        {
//                            bestY = newY;
//                            currentScore = newScore;
//                        }
//                    }
//                }
//                else
//                {
//                    // ������y�����ƶ���������Ż�λ��
//                    double currentScore = calculateScore(newPolygons);
//                    double newY = newPolygons[i].bbox().ymax();  // ��ǰλ�õ�y����
//                    double stepSize = 0.01;  // ����
//                    double bestY = newY;
//                    for (int k = 0; k < 10; ++k)
//                    {
//                        newY -= stepSize; // ������y���ϼ��ٲ���
//
//                        double newScore = calculateScore(newPolygons);
//                        // ����÷ָ��ã�������λ��
//                        if (newScore > currentScore)
//                        {
//                            bestY = newY;
//                            currentScore = newScore;
//                        }
//                    }
//                }
//                // ���ֺ�ѡ���еĴ�С�����������
//                if (nextCandidates.size() > beamWidth)
//                {
//                    nextCandidates.pop();
//                }
//            }
//        }
//        candidates = nextCandidates;
//    }
//    // ��ȡ��ѵĺ�ѡ�������
//    std::vector<Polygon_2> bestSolution = candidates.top().polygons;
//    // ����ѽ���������б�Ҫ�ĺ���
//    return bestSolution;
//}