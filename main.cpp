#include"Beamsearch.h"
#include"OutputWithLibigl.h"
int main()
{
	vector<vector<pair<Eigen::MatrixXd, Eigen::MatrixXi>>> ans;
	Beamsearch a;
	a.test();
	if (!a.process_solutions.empty()) {
		ans = Convert_Polygons_to_Matrix(a.process_solutions);
	}
	//OutputWithLibigl put;
	//put.get_score(a.GetScore());
	//put.Add_Polyons(ans);
	//std::cout << ans.size();
	//put.run();
}
