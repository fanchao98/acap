#ifndef VECTOR2
#define VECTOR2

#include <math.h>
#include <iostream>

//============================================================
//==== this file relize the operation of 2D vector
//==== Author:   Bo Zhitao
//==== Modifier: Rao Cong
//==== email:    raocongsdu@163.com
//============================================================


class Vector2
{
public:
	Vector2();
	Vector2(float x, float y);
	Vector2(const Vector2& v);
	Vector2(const float* v);
	~Vector2();

	float Length() const;
	void Normalized();
	void Output();


	Vector2& operator =  (const Vector2& a);
	Vector2& operator += (const Vector2& a);
	Vector2& operator += (float s);
	Vector2& operator -= (const Vector2& a);
	Vector2& operator -= (float s);
	Vector2& operator *= (float s);
	Vector2& operator /= (float s);


	friend Vector2 operator + (const Vector2& a, const Vector2& b);
	friend Vector2 operator + (const Vector2& a, float s);
	friend Vector2 operator + (float s, const Vector2& a);
	friend Vector2 operator - (const Vector2& a, const Vector2& b);
	friend Vector2 operator - (const Vector2& a, float s);
	friend Vector2 operator - (const Vector2& a);
	friend Vector2 operator * (const Vector2& a, float s);
	friend Vector2 operator * (float s, const Vector2& a);
	friend Vector2 operator / (const Vector2& a, float s);

	//Comparison
	friend bool operator == (const Vector2& a, const Vector2& b);
	friend bool operator != (const Vector2& a, const Vector2& b);
	friend bool operator <= (const Vector2& a, const Vector2& b);
	friend bool operator <  (const Vector2& a, const Vector2& b);
	friend bool operator >= (const Vector2& a, const Vector2& b);
	friend bool operator >  (const Vector2& a, const Vector2& b);


	friend float Cross(const Vector2& a, const Vector2& b);
	friend float Angle(const Vector2& a, const Vector2& b);
	friend float Dot(const Vector2& a, const Vector2& b);
	friend float Length(const Vector2& v);
	friend Vector2 Normalize(Vector2& v);


	float m_x, m_y;

};


#endif