#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <opencv2/opencv.hpp>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef CGAL::Polygon_2<Kernel> Polygon_2;
typedef CGAL::Polygon_with_holes_2<Kernel> Polygon_with_holes_2;

int main() {
    double a, b;
    int n;
    std::cin >> n;

    std::vector<cv::Point> points; // ���ڴ洢�������

    for (int i = 0; i < n; i++) {
        std::cin >> a >> b;
        cv::Point point(a+100, b+100);
        points.push_back(point); // ������ӵ�������
    }

    cv::Mat image(1000 + 10, 1000, CV_8UC3, cv::Scalar(0, 0, 0));

    // ���ƶ��������
    cv::polylines(image, points, true, cv::Scalar(255, 255, 255), 1);

    // ��ʾͼ��
    cv::imshow("name", image);
    cv::waitKey(0);

    return 0;
}

