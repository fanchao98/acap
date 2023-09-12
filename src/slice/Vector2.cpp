#include "Vector2.h"


Vector2::Vector2() : m_x(0.0), m_y(0.0)
{
}

Vector2::Vector2(float x, float y)
{
	this->m_x = x;
	this->m_y = y;
}

Vector2::Vector2(const Vector2& v)
{
	this->m_x = v.m_x;
	this->m_y = v.m_y;
}

Vector2::Vector2(const float* v)
{
	this->m_x = v[0];
	this->m_y = v[1];
}

Vector2::~Vector2()
{
}

float Vector2::Length() const
{
	float length = 0.0f;
	length = sqrtf((this->m_x * this->m_x) + (this->m_y * this->m_y));
	return length;
}

void Vector2::Normalized()
{
	float len = this->Length();
	if (len > 0.0f) {
		this->m_x = this->m_x / len;
		this->m_y = this->m_y / len;
	}
}

void Vector2::Output()
{
	std::cout << m_x << "  " << m_y << std::endl;
}




Vector2& Vector2::operator =  (const Vector2& a)
{
	this->m_x = a.m_x;
	this->m_y = a.m_y;
	return *this;
}
Vector2& Vector2::operator += (const Vector2& a)
{
	this->m_x += a.m_x;
	this->m_y += a.m_y;
	return *this;
}
Vector2& Vector2::operator += (float s)
{
	this->m_x += s;
	this->m_y += s;
	return *this;
}
Vector2& Vector2::operator -= (const Vector2& a)
{
	this->m_x -= a.m_x;
	this->m_y -= a.m_y;
	return *this;
}
Vector2& Vector2::operator -= (float s)
{
	this->m_x -= s;
	this->m_y -= s;
	return *this;
}
Vector2& Vector2::operator *= (float s)
{
	this->m_x *= s;
	this->m_y *= s;
	return *this;
}
Vector2& Vector2::operator /= (float s)
{
	this->m_x /= s;
	this->m_y /= s;
	return *this;
}


Vector2 operator + (const Vector2& a, const Vector2& b)
{
	Vector2 r;
	r.m_x = a.m_x + b.m_x;
	r.m_y = a.m_y + b.m_y;
	return r;
}
Vector2 operator + (const Vector2& a, float s)
{
	Vector2 r;
	r.m_x = a.m_x + s;
	r.m_y = a.m_y + s;
	return r;
}
Vector2 operator + (float s, const Vector2& a)
{
	Vector2 r;
	r.m_x = a.m_x + s;
	r.m_y = a.m_y + s;
	return r;
}
Vector2 operator - (const Vector2& a, const Vector2& b)
{
	Vector2 r;
	r.m_x = a.m_x - b.m_x;
	r.m_y = a.m_y - b.m_y;
	return r;
}
Vector2 operator - (const Vector2& a, float s)
{
	Vector2 r;
	r.m_x = a.m_x - s;
	r.m_y = a.m_y - s;
	return r;
}
Vector2 operator - (const Vector2& a)
{
	Vector2 r;
	r.m_x = -a.m_x;
	r.m_y = -a.m_y;
	return r;
}
Vector2 operator * (const Vector2& a, float s)
{
	Vector2 r;
	r.m_x = a.m_x * s;
	r.m_y = a.m_y * s;
	return r;
}
Vector2 operator * (float s, const Vector2& a)
{
	Vector2 r;
	r.m_x = a.m_x * s;
	r.m_y = a.m_y * s;
	return r;
}
Vector2 operator / (const Vector2& a, float s)
{
	Vector2 r;
	r.m_x = a.m_x / s;
	r.m_y = a.m_y / s;
	return r;
}

//Comparison
bool operator == (const Vector2& a, const Vector2& b)
{
	return(a.m_x == b.m_x && a.m_y == b.m_y);
}
bool operator != (const Vector2& a, const Vector2& b)
{
	return(a.m_x != b.m_x || a.m_y != b.m_y);
}
bool operator <= (const Vector2& a, const Vector2& b)
{
	return(a.m_x <= b.m_x && a.m_y <= b.m_y);
}
bool operator <  (const Vector2& a, const Vector2& b)
{
	return(a.m_x < b.m_x&& a.m_y < b.m_y);
}
bool operator >= (const Vector2& a, const Vector2& b)
{
	return(a.m_x >= b.m_x && a.m_y >= b.m_y);
}
bool operator >(const Vector2& a, const Vector2& b)
{
	return(a.m_x > b.m_x && a.m_y > b.m_y);
}







float Cross(const Vector2& a, const Vector2& b)
{
	return (a.m_x * b.m_y - a.m_y * b.m_x);
}


//�ж�����,���ݷ���������
float Angle(const Vector2& a, const Vector2& b)
{
	if (Length(a) < 0.000001 || Length(b) < 0.000001)
		return 0;

	float d = Dot(a, b);
	float al, bl;
	al = a.Length();
	bl = b.Length();

	float s = d / (al * bl);
	float angle = (float)acos((double)s);

	if (Cross(a, b) >= 0)
		return angle;
	else
		return -angle;

}

float Dot(const Vector2& a, const Vector2& b)
{
	float dot = 0.0f;
	dot = (a.m_x * b.m_x) + (a.m_y * b.m_y);
	return dot;
}

float Length(const Vector2& v)
{
	float length = 0.0f;
	length = sqrtf((v.m_x * v.m_x) + (v.m_y * v.m_y));
	return length;
}

Vector2 Normalize(Vector2& v)
{
	float len = v.Length();
	Vector2 norm;

	if (len > 0.0f) {
		norm.m_x = v.m_x / len;
		norm.m_y = v.m_y / len;
	}
	return norm;
}