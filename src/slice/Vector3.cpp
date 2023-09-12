#include "Vector3.h"


Vector3::Vector3() : m_x(0.0), m_y(0.0), m_z(0.0)
{

}

Vector3::Vector3(double x, double y, double z)
{
	this->m_x = x;
	this->m_y = y;
	this->m_z = z;
}

Vector3::Vector3(const Vector3& v)
{
	this->m_x = v.m_x;
	this->m_y = v.m_y;
	this->m_z = v.m_z;
}

Vector3::Vector3(const double* v)
{
	this->m_x = v[0];
	this->m_y = v[1];
	this->m_z = v[2];
}

Vector3::Vector3(double x)
{
	this->m_x = x;
	this->m_y = x;
	this->m_z = x;
}

Vector3::~Vector3()
{
}


double Vector3::Length() const
{
	double length = 0.0f;
	length = sqrtf((this->m_x * this->m_x) + (this->m_y * this->m_y) + (this->m_z * this->m_z));
	return length;
}

void Vector3::Normalized()
{
	double len = this->Length();
	if (len > 0.0f) {
		this->m_x = this->m_x / len;
		this->m_y = this->m_y / len;
		this->m_z = this->m_z / len;
	}
}

void Vector3::Output()
{
	std::cout << m_x << "  " << m_y << "  " << m_z << std::endl;
}

Vector3 Vector3::mulMatrix(double* matrix)
{
	Vector3 result;

	result.m_x = matrix[0] * this->m_x + matrix[1] * this->m_y + matrix[2] * this->m_z;
	result.m_y = matrix[3] * this->m_x + matrix[4] * this->m_y + matrix[5] * this->m_z;
	result.m_z = matrix[6] * this->m_x + matrix[7] * this->m_y + matrix[8] * this->m_z;

	return result;
}



//���ж�����,���ݷ���������
double Angle(const Vector3& a, const Vector3& b)
{
	double d = Dot(a, b);
	double al = a.Length();
	double bl = b.Length();

	double s = d / (al * bl);
	if (s > 1)
		s = 1;
	if (s < -1)
		s = -1;
	double angle = (double)acos(s);
	return angle;
}

double Dot(const Vector3& a, const Vector3& b)
{
	double dot = 0.0f;
	dot = (a.m_x * b.m_x) + (a.m_y * b.m_y) + (a.m_z * b.m_z);
	return dot;
}

double Length(const Vector3& v)
{
	double length = 0.0f;
	length = sqrtf((v.m_x * v.m_x) + (v.m_y * v.m_y) + (v.m_z * v.m_z));
	return length;
}

Vector3 Normalize(Vector3& v)
{
	double len = v.Length();
	Vector3 norm;
	norm.m_x = v.m_x / len;
	norm.m_y = v.m_y / len;
	norm.m_z = v.m_z / len;
	return norm;
}

Vector3 Cross(const Vector3& a, const Vector3& b)
{
	Vector3 cross;
	cross.m_x = (a.m_y * b.m_z) - (a.m_z * b.m_y);
	cross.m_y = (a.m_z * b.m_x) - (a.m_x * b.m_z);
	cross.m_z = (a.m_x * b.m_y) - (a.m_y * b.m_x);
	return cross;
}

double Triangle_area(Vector3& p, Vector3& q, Vector3& r)
{
	Vector3 U = q - p;
	Vector3 V = r - p;
	return Length(Cross(U, V)) / 2;
}




double nearest_point_on_triangle(     //rc
	const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3, Vector3& intp
) {
	//��p0��������p1p2p3�������intp;
	Vector3 N = Cross(p1 - p2, p3 - p2);
	N = Normalize(N);
	double min_dis = 10e8;

	/*Plane<T> P(p1, N);
	intp = line_plane_intersection(p0, N, P);*/
	intp = p0 + Dot(p1 - p0, N) * N;

	if (inside_triangle(intp, p1, p2, p3))
	{
		min_dis = Length(intp - p0);
		return min_dis;
	}
	else
	{
		Vector3 p_t[3] = { p1, p2, p3 };
		double temp_dis;
		for (int i = 0; i < 3; i++)
		{
			Vector3 va = p_t[i], vb = p_t[((i + 1) % 3)];
			Vector3 vab = vb - va, vac = p0 - va;
			double tf = Dot(vab, vac);
			if (tf < 0)
			{
				temp_dis = Length(p0 - va);
				if (temp_dis < min_dis)
				{
					min_dis = temp_dis;
					intp = va;
				}
				continue;
			}
			double td = Dot(vab, vab);

			if (tf > td)
			{
				temp_dis = Length(p0 - vb);
				if (temp_dis < min_dis)
				{
					min_dis = temp_dis;
					intp = vb;
				}
				continue;
			}
			tf = tf / td;
			Vector3 vD = va + tf * vab;   // c��ab�߶��ϵ�ͶӰ��
			temp_dis = Length(p0 - vD);
			if (temp_dis < min_dis)
			{
				min_dis = temp_dis;
				intp = vD;
			}
		}

		return min_dis;
	}
}

bool inside_triangle(const Vector3& p, const Vector3& a, const Vector3& b, const Vector3& c) {
	/*vec3g<T>& n = cross(b-a, c-a);
	T a0 = dot(cross(a-p, b-p), n);
	T a1 = dot(cross(b-p, c-p), n);
	T a2 = dot(cross(c-p, a-p), n);
	if(dot(cross(a-p, b-p), n)<-1e-10 || dot(cross(b-p, c-p), n)<-1e-10 || dot(cross(c-p, a-p), n)<-1e-10)
	return false;
	else
	return true;*/
	// Compute vectors        
	Vector3 v0 = c - a;
	Vector3 v1 = b - a;
	Vector3 v2 = p - a;

	// Compute dot products
	double	dot00 = Dot(v0, v0);
	double	dot01 = Dot(v0, v1);
	double	dot02 = Dot(v0, v2);
	double	dot11 = Dot(v1, v1);
	double	dot12 = Dot(v1, v2);

	// Compute barycentric coordinates
	double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return (u > 0) && (v > 0) && (u + v < 1);

}



//=====�Ƶ����̲ο��� http://blog.csdn.net/dlutxie/article/details/7746059 =============//
Vector3 Rotate(double rotate_angle, Vector3 v_norm, Vector3  v)
{
	v_norm.Normalized();
	double cosa = cos(rotate_angle);
	double sina = sin(rotate_angle);

	Vector3 v1 = Vector3(cosa + v_norm.m_x * v_norm.m_x * (1 - cosa), v_norm.m_x * v_norm.m_y * (1 - cosa) - v_norm.m_z * sina, v_norm.m_x * v_norm.m_z * (1 - cosa) + v_norm.m_y * sina);
	Vector3 v2 = Vector3(v_norm.m_y * v_norm.m_x * (1 - cosa) + v_norm.m_z * sina, cosa + v_norm.m_y * v_norm.m_y * (1 - cosa), v_norm.m_y * v_norm.m_z * (1 - cosa) - v_norm.m_x * sina);
	Vector3 v3 = Vector3(v_norm.m_z * v_norm.m_x * (1 - cosa) - v_norm.m_y * sina, v_norm.m_z * v_norm.m_y * (1 - cosa) + v_norm.m_x * sina, cosa + v_norm.m_z * v_norm.m_z * (1 - cosa));
	return Vector3(Dot(v1, v), Dot(v2, v), Dot(v3, v));
}


//Assignment
Vector3& Vector3::operator =  (const Vector3& a)
{
	this->m_x = a.m_x;
	this->m_y = a.m_y;
	this->m_z = a.m_z;
	return *this;
}
Vector3& Vector3::operator += (const Vector3& a)
{
	this->m_x += a.m_x;
	this->m_y += a.m_y;
	this->m_z += a.m_z;

	return *this;
}
Vector3& Vector3::operator += (double s)
{
	this->m_x += s;
	this->m_y += s;
	this->m_z += s;

	return *this;
}
Vector3& Vector3::operator -= (const Vector3& a)
{
	this->m_x -= a.m_x;
	this->m_y -= a.m_y;
	this->m_z -= a.m_z;

	return *this;
}
Vector3& Vector3::operator -= (double s)
{
	this->m_x -= s;
	this->m_y -= s;
	this->m_z -= s;

	return *this;
}
Vector3& Vector3::operator *= (double s)
{
	this->m_x *= s;
	this->m_y *= s;
	this->m_z *= s;

	return *this;
}
Vector3& Vector3::operator /= (double s)
{
	this->m_x /= s;
	this->m_y /= s;
	this->m_z /= s;

	return *this;
}





//Arithmetic
Vector3 operator + (const Vector3& a, const Vector3& b)
{
	Vector3 r;

	r.m_x = a.m_x + b.m_x;
	r.m_y = a.m_y + b.m_y;
	r.m_z = a.m_z + b.m_z;

	return r;
}
Vector3 operator + (const Vector3& a, double s)
{
	Vector3 r;

	r.m_x = a.m_x + s;
	r.m_y = a.m_y + s;
	r.m_z = a.m_z + s;

	return r;
}
Vector3 operator + (double s, const Vector3& a)
{
	Vector3 r;

	r.m_x = a.m_x + s;
	r.m_y = a.m_y + s;
	r.m_z = a.m_z + s;

	return r;
}
Vector3 operator - (const Vector3& a, const Vector3& b)
{
	Vector3 r;

	r.m_x = a.m_x - b.m_x;
	r.m_y = a.m_y - b.m_y;
	r.m_z = a.m_z - b.m_z;

	return r;
}
Vector3 operator - (const Vector3& a, double s)
{
	Vector3 r;

	r.m_x = a.m_x - s;
	r.m_y = a.m_y - s;
	r.m_z = a.m_z - s;

	return r;
}
Vector3 operator - (const Vector3& a)
{
	Vector3 r;

	r.m_x = -a.m_x;
	r.m_y = -a.m_y;
	r.m_z = -a.m_z;

	return r;
}
Vector3 operator * (const Vector3& a, double s)
{
	Vector3 r;

	r.m_x = a.m_x * s;
	r.m_y = a.m_y * s;
	r.m_z = a.m_z * s;

	return r;
}
Vector3 operator * (double s, const Vector3& a)
{
	Vector3 r;

	r.m_x = a.m_x * s;
	r.m_y = a.m_y * s;
	r.m_z = a.m_z * s;

	return r;
}
Vector3 operator / (const Vector3& a, double s)
{
	Vector3 r;

	r.m_x = a.m_x / s;
	r.m_y = a.m_y / s;
	r.m_z = a.m_z / s;

	return r;
}



Vector3 operator * (const Vector3& a, const Vector3& b)
{
	Vector3 r;

	r.m_x = a.m_x * b.m_x;
	r.m_y = a.m_y * b.m_y;
	r.m_z = a.m_z * b.m_z;

	return r;
}




//Comparison
bool operator == (const Vector3& a, const Vector3& b)
{
	return(a.m_x == b.m_x && a.m_y == b.m_y && a.m_z == b.m_z);
}
bool operator != (const Vector3& a, const Vector3& b)
{
	return(a.m_x != b.m_x || a.m_y != b.m_y || a.m_z != b.m_z);
}
bool operator <= (const Vector3& a, const Vector3& b)
{
	return(a.m_x <= b.m_x && a.m_y <= b.m_y && a.m_z <= b.m_z);
}
bool operator <  (const Vector3& a, const Vector3& b)
{
	return(a.m_x < b.m_x&& a.m_y < b.m_y&& a.m_z < b.m_z);
}
bool operator >= (const Vector3& a, const Vector3& b)
{
	return(a.m_x >= b.m_x && a.m_y >= b.m_y && a.m_z >= b.m_z);
}
bool operator >  (const Vector3& a, const Vector3& b)
{
	return(a.m_x > b.m_x && a.m_y > b.m_y && a.m_z > b.m_z);
}
