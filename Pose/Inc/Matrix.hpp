#pragma once
#ifndef __Matrix_H_
#define __Matrix_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>


#include <Eigen/Dense>

#define P_X	0
#define P_Y	1
#define P_Z	2

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

using namespace Eigen;

typedef struct
{
	std::vector<int> point;
	std::vector<Eigen::Vector2f> point2D;
	std::vector<Eigen::Vector3f> point3D;
} sp_point;

std::vector<Vector3f> slope_3d(Vector3f alpha, Vector3f omega);
Vector3f unit_vector(Vector3f vec);
Matrix4f coor(float roll, float pitch, float yaw, Vector3f point);

std::vector<Vector3f> slope_3d(Vector3f alpha, Vector3f omega);
Vector4f plane_vector(Vector3f a, Vector3f b, Vector3f c);
Vector3f nose_projection(Vector4f p_vector, std::vector<Vector3f> nose);

#else
#endif