#include <fstream>
#include "Robotics_config.h"
#include "dVRK_config.h"

int main()
{
	ifstream mtm_q_input("mtm_q.txt");
	if (!mtm_q_input)
	{
		cerr << "Fail to open the mtm_q input file! \n";
		return -1;
	}

	ofstream mtm_x_out("mtm_x.txt");
	if (!mtm_x_out)
	{
		cerr << "Fail to open the mtm_x output file! \n";
		return -2;
	}

	MatrixXf mtm_q_cur = MatrixXf::Zero(MTM_frame_num, 1);
	Matrix4f MTM_Trasf = Matrix4f::Identity();
	DH MTM(MTM_frame_num);
	float ang = 0.0F;
	int idx = 0;

	// *** Test on the forward kinematics of MTM. *** //
	while (mtm_q_input >> ang)
	{
		mtm_q_cur.row(idx) << ang;
		if (idx == MTM_frame_num - 1)
		{
			MTM = MTM_Model(mtm_q_cur);
			MTM_Trasf = MTM_FK(MTM);
			mtm_x_out << MTM_Trasf << '\n' << '\n';
			//cout << "MTM_Tip Position:\n" << MTM_Trasf << endl;
			idx = 0;
		}
		else
		++idx;
	}


	//cout << mtm_q_cur << endl;
	mtm_q_input.close();
	mtm_x_out.close(); 

	/*MatrixXf temp1(2, 3);
	temp1 << 1, 1, 1, 3, 3, 3;
	JacobiSVD<MatrixXf> svd_A(temp1, ComputeFullU | ComputeFullV);
	cout << svd_A.singularValues() << endl;*/
}