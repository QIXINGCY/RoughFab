//����һ��ͼ�����ģ��
//�Ķ�ǰ���Ȳ鿴beamsearch.h
//AD�л�ͼ��WS�����ƶ�ѡ��ͼ�Σ�QE�л�ѡ��ͼ�Σ�RF�����ƶ�
//��������ʾ���洢����ͼ����Ϣ��ͨ��������Ӧ����ͼ���״̬���Ƿ�ɼ�����ɫ��λ��
//ע�������ʽ
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <iostream>
class OutputWithLibigl {
public:
    void Add_Polyons(std::vector<std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>> input);//����ͼ����Ϣ������vector
    void run();//���ӻ�����
    void get_score(std::vector<double> a);//��ȡ�÷�
private:
    igl::opengl::glfw::Viewer viewers;//��ʾ��
    std::vector<int> FirstNum;//��¼
    std::vector<double> score;//�÷�
    int viewer_num;//ͼ������
    int now_viewer;//��¼��ǰͼ��
    int now_polyon;//��¼��ǰѡ��ͼ��
    bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier);//������Ӧ
};
void OutputWithLibigl::get_score(std::vector<double> a) {//vector�洢�ĵ÷�
    score = a;
}
void OutputWithLibigl::Add_Polyons(std::vector<std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>>> input) {
    viewer_num = 0;
    now_viewer = 0;
    now_polyon = 0;
    igl::opengl::ViewerData data;//��ʾ���洢�Ļ���Ԫ��
    int num = 0;
    for (auto i : input) {
        FirstNum.push_back(viewer_num);//��¼ÿ���һ��ͼ�������
        for (auto j : i) {
            data.clear();//��ʼ��
            data.set_mesh(j.first, j.second);//��������
            data.show_overlay_depth = false; // ������ȸ���
            data.is_visible = false; // ��ʼ״̬�²��ɼ�
            data.face_based = true; // ʹ������ɫ�����Ƕ�����ɫ
            data.set_colors(Eigen::RowVector3d(1, 0, 0)); // ������ɫΪ��ɫ
            viewer_num++;
            viewers.data_list.push_back(data);//������ʾ��
        }
    }
    FirstNum.push_back(viewer_num);//��¼���һ��ͼ�������
}
bool OutputWithLibigl::key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
    std::cout << "Key: " << key << " " << (unsigned int)key << std::endl;
    if (key == 65)//��ֵ����Ӧ��ĸ//a
    {//�����Ƿ�ɼ����Ժ���ɫ
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
            std::cout << "�Ѿ��ǵ�һ��ͼƬ��" << std::endl;
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
            std::cout << "�Ѿ������һ��ͼƬ��" << std::endl;
        }
    }
    else if (key == 87)//s
    {//���ݴ���
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
    {//��ɫ����Խ����
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
    {//����ƶ�
        viewer.core().camera_translation += Eigen::Vector3f(0, 10, 0);//����ƶ�
    }
    else if (key == 70)//f
    {
        viewer.core().camera_translation -= Eigen::Vector3f(0, 10, 0);//����ƶ�
    }
    //�����Ϣ
    std::cout << "��ǰ�ǵ�" << now_viewer + 1 << "���" << "��" << now_polyon - FirstNum[now_viewer] + 1 << "��ͼ��" << std::endl;
    std::cout << "��ǰ�÷�Ϊ��" << score[now_polyon] << std::endl;
    return false;
}
void OutputWithLibigl::run() {
    viewers.callback_key_down = std::bind(&OutputWithLibigl::key_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    // �����鿴��
    viewers.data_list[now_polyon + 1].set_colors(Eigen::RowVector3d(0, 0, 1));//ѡ�е�һ��ͼ��
    for (int i = FirstNum[0]; i < FirstNum[1]; i++) {//��ʼ������һ��ͼ��Ϊ�ɼ�
        viewers.data_list[i + 1].is_visible = true;
    }
    viewers.launch();
}