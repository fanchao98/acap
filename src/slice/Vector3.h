#ifndef VECTOR3
#define VECTOR3

#include <math.h>
#include <iostream>



class Vector3
{
public:
	Vector3();
	Vector3(double x, double y, double z);
	Vector3(const Vector3& v);
	Vector3(const double* v);
	Vector3(double x);
	~Vector3();


	double Length() const;
	void Normalized();
	void Output();
	Vector3 mulMatrix(double* matrix);


	friend double Angle(const Vector3& a, const Vector3& b);
	friend double Dot(const Vector3& a, const Vector3& b);
	friend double Length(const Vector3& v);
	friend Vector3 Normalize(Vector3& v);
	friend Vector3 Cross(const Vector3& a, const Vector3& b);
	friend Vector3 Rotate(double rotate_angle, Vector3 v_norm, Vector3  v);
	friend double Triangle_area(Vector3& p, Vector3& q, Vector3& r);


	friend double nearest_point_on_triangle(     //rc
		const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3, Vector3& intp);
	friend bool inside_triangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c);



	//Assignment
	Vector3& operator =  (const Vector3& a);
	Vector3& operator += (const Vector3& a);
	Vector3& operator += (double s);
	Vector3& operator -= (const Vector3& a);
	Vector3& operator -= (double s);
	Vector3& operator *= (double s);
	Vector3& operator /= (double s);

	//Arithmetic
	friend Vector3 operator + (const Vector3& a, const Vector3& b);
	friend Vector3 operator + (const Vector3& a, double s);
	friend Vector3 operator + (double s, const Vector3& a);
	friend Vector3 operator - (const Vector3& a, const Vector3& b);
	friend Vector3 operator - (const Vector3& a, double s);
	friend Vector3 operator - (const Vector3& a);
	friend Vector3 operator * (const Vector3& a, const Vector3& b);
	friend Vector3 operator * (const Vector3& a, double s);
	friend Vector3 operator * (double s, const Vector3& a);
	friend Vector3 operator / (const Vector3& a, const Vector3& b);
	friend Vector3 operator / (const Vector3& a, double s);

	//Comparison
	friend bool operator == (const Vector3& a, const Vector3& b);
	friend bool operator != (const Vector3& a, const Vector3& b);
	friend bool operator <= (const Vector3& a, const Vector3& b);
	friend bool operator <  (const Vector3& a, const Vector3& b);
	friend bool operator >= (const Vector3& a, const Vector3& b);
	friend bool operator >  (const Vector3& a, const Vector3& b);

	double m_x;
	double m_y;
	double m_z;
};

#endif