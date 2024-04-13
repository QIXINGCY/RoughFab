//这是一个图形输出模块
//阅读前请先查看beamsearch.h
//AD切换图像，WS上下移动选中图形，QE切换选中图形，RF整体移动
//方案是显示器存储所有图像信息，通过键盘响应更改图像的状态：是否可见，颜色，位置
//注意输入格式
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
class OutputWithLibigl {
public:
    void Add_Polyons(std::vector<std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>> input);//输入图像信息，两层vector
    void run();//可视化运行
    void get_score(std::vector<double> a);//获取得分
private:
    igl::opengl::glfw::Viewer viewers;//显示器
    std::vector<int> FirstNum;//记录
    std::vector<double> score;//得分
    int viewer_num;//图像数量
    int now_viewer;//记录当前图像
    int now_polyon;//记录当前选中图形
    bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier);//键盘响应
};
void OutputWithLibigl::get_score(std::vector<double> a) {//vector存储的得分
    score = a;
}
void OutputWithLibigl::Add_Polyons(std::vector<std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>> input) {
    viewer_num = 0;
    now_viewer = 0;
    now_polyon = 0;
    igl::opengl::ViewerData data;//显示器存储的基本元素
    int num = 0;
    for (auto i : input) {
        FirstNum.push_back(viewer_num);//记录每层第一个图像的索引
        for (auto j : i) {
            data.clear();//初始化
            data.set_mesh(j.first, j.second);//加入数据
            data.show_overlay_depth = false; // 禁用深度覆盖
            data.is_visible = false; // 初始状态下不可见
            data.face_based = true; // 使用面颜色而不是顶点颜色
            data.set_colors(Eigen::RowVector3d(1, 0, 0)); // 设置颜色为红色
            viewer_num++;
            viewers.data_list.push_back(data);//放入显示器
        }
    }
    FirstNum.push_back(viewer_num);//记录最后一个图像的索引
}
bool OutputWithLibigl::key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
    std::cout << "Key: " << key << " " << (unsigned int)key << std::endl;
    if (key == 65)//键值，对应字母//a
    {//更改是否可见属性和颜色
        if (now_viewer > 0) {
            viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(1, 0, 0));
            for (int i = FirstNum[now_viewer]; i < FirstNum[now_viewer + 1]; i++) {
                viewers.data_list[i + 1].is_visible = false;
            }
            for (int i = FirstNum[now_viewer - 1]; i < FirstNum[now_viewer]; i++) {
                viewers.data_list[i + 1].is_visible = true;
            }
            now_viewer--;
            now_polyon = FirstNum[now_viewer];
            viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));
        }
        else {
            std::cout << "已经是第一张图片了" << std::endl;
        }
    }
    else if (key == 68)//d
    {
        if (now_viewer < viewer_num - 1) {
            viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(1, 0, 0));
            for (int i = FirstNum[now_viewer]; i < FirstNum[now_viewer + 1]; i++) {
                viewers.data_list[i + 1].is_visible = false;
            }
            for (int i = FirstNum[now_viewer + 1]; i < FirstNum[now_viewer + 2]; i++) {
                viewers.data_list[i + 1].is_visible = true;
            }
            now_viewer++;
            now_polyon = FirstNum[now_viewer];
            viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));
        }
        else {
            std::cout << "已经是最后一张图片了" << std::endl;
        }
    }
    else if (key == 87)//s
    {//数据处理
        for (int i = 0; i < viewers.data_list[now_polyon + 1].V.rows(); ++i) {
            viewers.data_list[now_polyon + 1].V(i, 1) += 10;
        }
        viewers.data_list[now_polyon + 1].set_mesh(viewers.data_list[now_polyon + 1].V, viewers.data_list[now_polyon + 1].F);
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));

    }
    else if (key == 83)//w
    {
        for (int i = 0; i < viewers.data_list[now_polyon + 1].V.rows(); ++i) {
                viewers.data_list[now_polyon + 1].V(i, 1)-=10;
        }
        viewers.data_list[now_polyon + 1].set_mesh(viewers.data_list[now_polyon + 1].V, viewers.data_list[now_polyon + 1].F);
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));
    }
    else if (key == 81)//q
    {//颜色处理，越界检测
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(1, 0, 0));
        now_polyon--;
        if (now_polyon < FirstNum[now_viewer]) {
            now_polyon = FirstNum[now_viewer + 1] - 1;
        }
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));
    }
    else if (key == 69)//e
    {
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(1, 0, 0));
        now_polyon++;
        if (now_polyon >= FirstNum[now_viewer + 1]) {
            now_polyon = FirstNum[now_viewer];
        }
        viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));
    }
    else if (key == 82)//r
    {//相机移动
        viewer.core().camera_translation += Eigen::Vector3f(0, 10, 0);//相机移动
    }
    else if (key == 70)//f
    {
        viewer.core().camera_translation -= Eigen::Vector3f(0, 10, 0);//相机移动
    }
    //输出信息
    std::cout << "当前是第" << now_viewer + 1 << "层的" << "第" << now_polyon - FirstNum[now_viewer] + 1 << "个图形" << std::endl;
    std::cout << "当前得分为：" << score[now_polyon] << std::endl;
    return false;
}
void OutputWithLibigl::run() {
    viewers.callback_key_down = std::bind(&OutputWithLibigl::key_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    // 启动查看器
    viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));//选中第一个图形
    for (int i = FirstNum[0]; i < FirstNum[1]; i++) {//初始化，第一个图像为可见
        viewers.data_list[i + 1].is_visible = true;
    }
    viewers.launch();
}