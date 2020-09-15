#include "Matrix.hpp"


std::vector<Vector3f> slope_3d(Vector3f alpha, Vector3f omega)
{
	std::vector<Vector3f> vec;

	vec.push_back(Vector3f((omega[P_X] + alpha[P_X]) / 2, (omega[P_Y] + alpha[P_Y]) / 2, (omega[P_Z] + alpha[P_Z]) / 2));

	return vec;
}

Vector3f unit_vector(Vector3f vec)
{
	float d = sqrtf(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2));
	Vector3f return_vector(vec[0] / d, vec[1] / d, vec[2] / d);

	return return_vector;
}

Matrix4f coor(float roll, float pitch, float yaw, Vector3f point)
{

	Matrix4f Trans(4, 4);
	Matrix3f Rx;
	Rx << 1, 0, 0,
		0, cosf(roll), -sinf(roll),
		0, sinf(roll), cosf(roll);

	Matrix3f Ry;
	Ry << cosf(pitch), 0, sinf(pitch),
		0, 1, 0
		- sin(pitch), 0, cos(pitch);

	Matrix3f Rz;
	Rz << cosf(yaw), -sinf(yaw), 0,
		sinf(yaw), cosf(yaw), 0,
		0, 0, 1;

	Matrix3f R = Rz * Ry * Rx;

	Trans.block<3, 3>(0, 0) = R;
	
	return Trans;
}

Vector4f plane_vector(Vector3f a, Vector3f b, Vector3f c)
{
	Vector3f first_vector(b[P_X] - a[P_X], b[P_Y] - a[P_Y], b[P_Z] - a[P_Z]);
	Vector3f second_vector(c[P_X] - a[P_X], c[P_Y] - a[P_Y], c[P_Z] - a[P_Z]);

	Vector3f n_vector = first_vector.cross(second_vector);

	float d = n_vector[P_X] * a[P_X] + n_vector[P_Y] * a[P_Y] + n_vector[P_Z] * a[P_Z];

	Vector4f p_vector(n_vector[P_Z], n_vector[P_Y], n_vector[P_Z], d);

	return p_vector;
}

Vector3f nose_projection(Vector4f p_vector, std::vector<Vector3f> nose)
{

	int i;
	Vector3f n_vector;

	for (i = 0; i < 3; i++)
		n_vector[i] = p_vector[i];

	n_vector = nose.at(0) - abs(n_vector.dot(nose.at(0)) + p_vector[3]) * n_vector;		//p = q - distance * n

	return n_vector;
}