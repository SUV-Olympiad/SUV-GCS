#include "mymath.h"

float doublePI(float value)
{
    if ( value < 0 ) {
        return M_PI*2 + value;
    }
    else {
        return value;
    }
}



// Jang added 2015. 11. 13.
Eigen::Vector3f rotVec(Eigen::Vector3f a, Eigen::Vector3f b)
{
	// Name: Rotation Vector of two vectors
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: two vector
	// Output: the rotation vector from a to b
	// Purpose: To get the rotation vector from a to b

	Eigen::Vector3f r; // the rotation vector from a to b
	static Eigen::Vector3f unit_prev(0, 0, 0); // the previous unit vector of the output
	/* Bug: the static variable above is for all the calls so problems will happens for multiple flights */

	Eigen::Vector3f outer = a.cross(b); //the outer product of a and b
	float inner = a.dot(b); // the inner product of a anb b

	float mag_a = a.norm(); // the magnitude of a
	float mag_b = b.norm(); // the magnitude of b

	float mag_outer = outer.norm(); // the magnitude of the outer product

	if(mag_a == 0 || mag_b == 0) // || mag_outer == 0) // for singular point
	{
		r = Eigen::Vector3f(0, 0, 0);
	}
	else
	{
		float sine = mag_outer / mag_a / mag_b; // the sine of the angle
		float cosine = inner / mag_a / mag_b; // the cosine of the angle

		float theta = atan2(sine, cosine); // the angble between a and b

		Eigen::Vector3f unit; // the unit vector

		if(mag_outer == 0) // for singularity at opposite vectors
		{
			if(unit_prev != Eigen::Vector3f(0, 0, 0)) // when a previous unit vector exists
			{
				unit = unit_prev; // let's use the previous unit vector
			}
			else // where any previous unit vector does not exist
			{
				unit = Eigen::Vector3f(1, 0, 0); // let x axis be the unit vector
			}
		}
		else
		{
			unit = outer / mag_outer; // the unit vector of the outer product
		}

		unit_prev = unit;

		r = theta * unit; // the rotation vector from a to b
	}

	return r;
}

// Jang added 2015. 11. 13.
float angle(Eigen::Vector3f a, Eigen::Vector3f b)
{
	// Name: Angle of two vectors
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: two vector
	// Output: the angle between a and b (-¥ð to ¥ð rad)
	// Purpose: To get the angle between a and b

	float y; // the angle between a and b

	Eigen::Vector3f outer = a.cross(b); // the outer product of a and b
	float inner = a.dot(b); // the inner product of a anb b

	float mag_a = a.norm(); // the magnitude of a
	float mag_b = b.norm(); // the magnitude of b

	float mag_outer = outer.norm(); // the magnitude of the outer product

	if(mag_a == 0 || mag_b == 0) // for singular point
	{
		y = 0;
	}
	else
	{
		float sine = -sign(outer(2)) * mag_outer / mag_a / mag_b; // the sine of the angle
		float cosine = inner / mag_a / mag_b; // the cosine of the angle

		y = atan2(sine, cosine); // the angble between a and b
	}

	return y;
}

Eigen::Matrix3f rodrigues(Eigen::Vector3f v)
{
	// Name: Rodrigues' Rotation Formula
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: a rotation vector
	// Output: a rotation matrix
	// Purpose: To convert a rotation vector to a rotation matrix
	// This algorithm uses Rodrigues' rotation formula

	Eigen::Matrix3f R; // the rotatoin matrix of the v
	Eigen::Matrix3f I = Eigen::Matrix3f::Identity(); // the identity matrix

	float theta = v.norm(); // the rotation angle of the rotation vector v

	if(theta == 0) // for singular point
	{
		R = I;
	}
	else
	{
		Eigen::Vector3f u = v / theta; // the unit vector of the rotation vector v

		Eigen::Matrix3f U; // the skew symmetric matrix of u

		U << 0,   -u(2),  u(1),
			u(2),   0,   -u(0),
		   -u(1),  u(0),    0;

		// To use the Rodrigues' rotation formula
		R = I + qSin(theta)*U + (1 - qCos(theta))*U*U;
	}

	return R;
}

double yaw3D(Eigen::Matrix3f R)
{
	// Name: yaw in 3D space
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: a rotation matrix of a body
	// Output: a yaw angle in 3D space
	// Purpose: To get the yaw angle rotationaly projected on the horizontal plane of the inertial frame
	// This algorithm was invented newly from a research
	// R: the rotation matrix of the body

	double y; // the yaw angle

	Eigen::Vector3f X(1, 0, 0); // the X axis unit vector of the inertial frame
	Eigen::Vector3f Z(0, 0, 1); // the Z axis unit vector of the inertial frame
	Eigen::Vector3f x; // the x axis unit vector of the body frame
	Eigen::Vector3f z; // the z axis unit vector of the body frame
	Eigen::Vector3f x_p; // the rotationaly projected vector of the x onto the horizontal plane of the inertial frame

	x = Eigen::Vector3f(R(0, 0), R(1, 0), R(2, 0));
	z = Eigen::Vector3f(R(0, 2), R(1, 2), R(2, 2));

	x_p = rodrigues(rotVec(z, Z))*x;

	y = angle(x_p, X);

	return y;
}

int sign(float x)
{
	// Name: Sign
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: a scalar
	// Output: an integer
	// Purpose: To get the sign of a scalar

	int y;

	if(x > 0)
	{
		y = 1;
	}
	else if(x < 0)
	{
		y = -1;
	}
	else if(x == 0)
	{
		y = 0;
	}

	return y;
}

float trim(float x)
{
	// Name: Trim the range of the angle
	// Author: Jang, Jong Tai 2015. 11. 13.
	// Input: an angle [rad]
	// Output: the angle trimmed in the range of -pi to pi rad
	// Purpose: To trim the angle in the range of -pi to pi rad

	float y; // the trimmed angle

	if(x < -M_PI)
	{
		y = x + 2*M_PI;
	}
	else if( x > M_PI)
	{
		y = x - 2*M_PI;
	}

	return y;
}

Eigen::Matrix3f rotateX(float roll)
{
	// Input: the angle [rad] of the y axis from the Y axis around the X axis
	// Output: the rotation matrix of the body frame from the world rame

	Eigen::Matrix3f R;

	R << 1,       0,          0,
		 0,   qCos(roll), -qSin(roll),
		 0,   qSin(roll),  qCos(roll);

	return R;
}

Eigen::Matrix3f rotateY(float pitch)
{
	// Input: the angle [rad] of the x axis from the X axis around the Y axis
	// Output: the rotation matrix of the body frame from the world rame

	Eigen::Matrix3f R;

	R << qCos(pitch),   0,   qSin(pitch),
			  0,        1,        0,
		-qSin(pitch),   0,   qCos(pitch);

	return R;
}

Eigen::Matrix3f rotateZ(float yaw)
{
	// Input: the angle [rad] of the x axis from the X axis around the Z axis
	// Output: the rotation matrix of the body frame from the world rame

	Eigen::Matrix3f R;

	R << qCos(yaw), -qSin(yaw),   0,
		 qSin(yaw),  qCos(yaw),   0,
			 0,         0,        1;

	return R;
}


// refer : https://bowbowbow.tistory.com/17
double ccw(QVector2D a, QVector2D b)
{
    return a.x() * b.y() - a.y()*b.x();
}

// refer : https://bowbowbow.tistory.com/17
double ccw(QVector2D p, QVector2D a, QVector2D b)
{
    return ccw(a-p, b-p);
}

// refer : https://bowbowbow.tistory.com/17
bool sementIntersects(QVector2D a, QVector2D b, QVector2D c, QVector2D d)
{
    double ab = ccw(a, b, c)*ccw(a, b, d);
    double cd = ccw(c, d ,a)*ccw(c, d, b);

    if(ab ==0 && cd == 0){
        QVector2D temp;

        if( b.x() < a.x() && b.y() < a.y() ) {
            temp = a;
            a = b;
            b = temp;
        }
        if( d.x() < c.x() && d.y() < c.y() ) {
            temp = c;
            c = d;
            d = temp;
        }
        return !( (b.x() < c.x() && b.y() < c.y()) || (d.x() < a.x() && d.y() < a.y()));
//        return !( b< c || d < a);
    }
    return ab <= 0 && cd <=0;
}

