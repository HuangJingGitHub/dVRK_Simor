#include <iostream>
#include <cmath>
#include "Eigen\Dense"

using namespace std;
using namespace Eigen;

const float pi = 3.141592F;
enum JointType {revolute = 1, prismatic};
typedef JointType Joint;

class DH {
public:
	DH(int frame_num);
	DH(MatrixXf input_DH)
	{
		if (input_DH.cols() != 5)
			cerr << "Invalid DH table size! NOTE: DH table should have 5 columns.";
		else
		{
			_frame_num = input_DH.rows();
			_DH = input_DH;
		}
	}
	bool DH_set(int row, int col, float val);
	bool DH_set(MatrixXf DH_table);
	void DH_read() const;
	MatrixXf DH_get() const;
	MatrixXi DH_size() const;
	int DH_row() const
	{
		return _frame_num;
	}
	ostream& operator<<(ostream& os) const;

private:
	int _frame_num;
	MatrixXf _DH;
};

DH::DH(int frame_num)
{
	_frame_num = frame_num;
	_DH = MatrixXf::Zero(frame_num, 5);
}

bool DH::DH_set(int row, int col, float val)
{
	if (row < 0 || row > _frame_num - 1 || col < 0 || col > 4)
	{
		cerr << "Invalid index used for DH table !!!\n" << endl;
		return false;
	}
	else
		_DH(row, col) = val;
	return true;
}

bool DH::DH_set(MatrixXf DH_table)
{
	_DH = DH_table;
	_frame_num = _DH.rows();
	return true;
}

void DH::DH_read() const
{
	cout << _DH << endl;
}

MatrixXf DH::DH_get() const
{
	return _DH;
}
MatrixXi DH::DH_size() const
{
	MatrixXi DH_size(2, 1);
	DH_size(1, 1) = _frame_num;
	DH_size(2, 1) = 5;
	return DH_size;
}

ostream& DH::operator<<(ostream& os) const
{
	os << _DH << endl;
	return os;
}


//*** Defination of class Kinematics_info ***
class Kinematics_info
{
public:
	Kinematics_info(int frame_num)
	{
		_frame_num = frame_num;
		_T_tip = MatrixXf::Zero(4, 4);
		_T_tip(3, 3) = 1;
		_origin_pos = MatrixXf::Zero(3, frame_num);
		_Jacobian = MatrixXf::Zero(6, frame_num - 1);
	}
	MatrixXf tip_get() const
	{	return _T_tip;	}
    void  tip_set(Matrix4f& tip) 
	{  _T_tip = tip;	}

	MatrixXf origin_pos_get() const
	{	return _origin_pos;	}
	void origin_pos_set(MatrixXf pos)
	{	_origin_pos = pos;	}

	MatrixXf J_get() const
	{	return _Jacobian; }
	void J_set(MatrixXf j)
	{	_Jacobian = j;	}
private:
	int _frame_num;
	Matrix4f _T_tip;
	MatrixXf _origin_pos;
	MatrixXf _Jacobian;
};

MatrixXf vel_cal(Matrix4f& T_cur, Matrix4f& T_pre, float dt)  // Pass fixed size matrix in eigen, use reference.
{
	MatrixXf vel = MatrixXf::Zero(6, 1);
	Matrix3f R_cur = T_cur.block(0, 0, 3, 3),
			 R_pre = T_pre.block(0, 0, 3, 3);
	Matrix3f T = R_cur * R_pre.transpose();
	vel(3, 0) = T(2, 1) - T(1, 2);
	vel(4, 0) = T(0, 2) - T(2, 0);
	vel(5, 0) = T(1, 0) - T(0, 1);

	/*double epsilon = 1e-5,
		  theta = acos(0.5*(T.trace() - 1));
	if (theta > epsilon && theta < pi - epsilon)
		vel = vel * (theta / 2 / sin(theta));
	else
		vel = vel * (theta / 2 / sin(epsilon));*/
	
	vel(0, 0) = T_cur(0, 3) - T_pre(0, 3);
	vel(1, 0) = T_cur(1, 3) - T_pre(1, 3);
	vel(2, 0) = T_cur(2, 3) - T_pre(2, 3);
	vel = vel / dt;
	return vel;
}

MatrixXf xdif(Matrix4f& T_cur, Matrix4f& T_pre)
{
	MatrixXf xerr = MatrixXf::Zero(6, 1);
	Matrix3f R_cur = T_cur.block(0, 0, 3, 3),
			 R_pre = T_pre.block(0, 0, 3, 3);
	Matrix3f T = R_cur * R_pre.transpose();
	xerr(3, 0) = T(2, 1) - T(1, 2);
	xerr(4, 0) = T(0, 2) - T(2, 0);
	xerr(5, 0) = T(1, 0) - T(0, 1);

	/*double epsilon = 1e-5,
		theta = acos(0.5*(T.trace() - 1));
	if (theta > epsilon && theta < pi - epsilon)
		xerr = xerr * (theta / 2 / sin(theta));
	else
		xerr = xerr * (theta / 2 / sin(epsilon));*/

	xerr(0, 0) = T_cur(0, 3) - T_pre(0, 3);
	xerr(1, 0) = T_cur(1, 3) - T_pre(1, 3);
	xerr(2, 0) = T_cur(2, 3) - T_pre(2, 3);
	return xerr;
}

MatrixXf right_pseudoinv(MatrixXf A)
{
	MatrixXf rig_pesuinv;
	int row_A = A.rows(),
		col_A = A.cols(),
		m = min(row_A, col_A);
	JacobiSVD<MatrixXf> svd_A(A, ComputeFullU | ComputeFullV);
	MatrixXf U = svd_A.matrixU(),
			 V = svd_A.matrixV(),
			 singval = svd_A.singularValues(),
			 S = MatrixXf::Zero(row_A, col_A);

	for (int i = 0; i < m; i++)
		S(i, i) = singval(i, 0);

	MatrixXf SS_T = S*(S.transpose());
	rig_pesuinv = V * S.transpose() * SS_T.inverse() * U.transpose();
	return rig_pesuinv;
}
