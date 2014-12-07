#ifndef I_COMMON_H
#define I_COMMON_H

#include "math.h"
#include "stdlib.h"

typedef unsigned int Pixel;

inline float Rand( float a_Range ) { return ((float)rand() / RAND_MAX) * a_Range; }

namespace Raytracer {


#define TRACEDEPTH		4
#define MAXTREEDEPTH	20
#define IMPORTANCE
#define TILESIZE		16
// #define HIGHPRECISION

#ifndef HIGHPRECISION
#define EPSILON			0.0001f
#define real	float
#define _fabs	fabsf
#define _cos	cosf
#define _sin	sinf
#define _acos	acosf
#define _floor	floorf
#define _ceil	ceilf
#define _sqrt	sqrtf
#define _pow	powf
#define _exp	expf
#else
#define EPSILON			0.0000001f
#define real	double
#define _fabs	fabs
#define _cos	cos
#define _sin	sin
#define _acos	acos
#define _floor	floor
#define _ceil	ceil
#define _sqrt	sqrt
#define _pow	pow
#define _exp	exp
#endif

#define DOT(A,B)		(A.x*B.x+A.y*B.y+A.z*B.z)
#define NORMALIZE(A)	{real l=1/_sqrt(A.x*A.x+A.y*A.y+A.z*A.z);A.x*=l;A.y*=l;A.z*=l;}
#define LENGTH(A)		(_sqrt(A.x*A.x+A.y*A.y+A.z*A.z))
#define SQRLENGTH(A)	(A.x*A.x+A.y*A.y+A.z*A.z)
#define SQRDISTANCE(A,B) ((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y)+(A.z-B.z)*(A.z-B.z))

#define PI				3.141592653589793238462f

class vector3
{
public:
	vector3() : x( 0.0f ), y( 0.0f ), z( 0.0f ) {};
	vector3( real a_X, real a_Y, real a_Z ) : x( a_X ), y( a_Y ), z( a_Z ) {};
	void Set( real a_X, real a_Y, real a_Z ) { x = a_X; y = a_Y; z = a_Z; }
	void Normalize() { real l = 1.0f / Length(); x *= l; y *= l; z *= l; }
	real Length() { return (real)sqrt( x * x + y * y + z * z ); }
	real SqrLength() { return x * x + y * y + z * z; }
	real Dot( vector3 a_V ) { return x * a_V.x + y * a_V.y + z * a_V.z; }
	vector3 Cross( vector3 b ) { return vector3( y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x ); }
	void operator += ( vector3& a_V ) { x += a_V.x; y += a_V.y; z += a_V.z; }
	void operator += ( vector3* a_V ) { x += a_V->x; y += a_V->y; z += a_V->z; }
	void operator -= ( vector3& a_V ) { x -= a_V.x; y -= a_V.y; z -= a_V.z; }
	void operator -= ( vector3* a_V ) { x -= a_V->x; y -= a_V->y; z -= a_V->z; }
	void operator *= ( real f ) { x *= f; y *= f; z *= f; }
	void operator *= ( vector3& a_V ) { x *= a_V.x; y *= a_V.y; z *= a_V.z; }
	void operator *= ( vector3* a_V ) { x *= a_V->x; y *= a_V->y; z *= a_V->z; }
	vector3 operator- () const { return vector3( -x, -y, -z ); }
	friend vector3 operator + ( const vector3& v1, const vector3& v2 ) { return vector3( v1.x + v2.x, v1.y + v2.y, v1.z + v2.z ); }
	friend vector3 operator - ( const vector3& v1, const vector3& v2 ) { return vector3( v1.x - v2.x, v1.y - v2.y, v1.z - v2.z ); }
	friend vector3 operator + ( const vector3& v1, vector3* v2 ) { return vector3( v1.x + v2->x, v1.y + v2->y, v1.z + v2->z ); }
	friend vector3 operator - ( const vector3& v1, vector3* v2 ) { return vector3( v1.x - v2->x, v1.y - v2->y, v1.z - v2->z ); }
	friend vector3 operator * ( const vector3& v, real f ) { return vector3( v.x * f, v.y * f, v.z * f ); }
	friend vector3 operator * ( const vector3& v1, vector3& v2 ) { return vector3( v1.x * v2.x, v1.y * v2.y, v1.z * v2.z ); }
	friend vector3 operator * ( real f, const vector3& v ) { return vector3( v.x * f, v.y * f, v.z * f ); }
	union
	{
		struct { real x, y, z; };
		struct { real r, g, b; };
		struct { real entry[3]; };
	};
};

class matrix
{
public:
	enum 
	{ 
		TX=3, 
		TY=7, 
		TZ=11, 
		D0=0, D1=5, D2=10, D3=15, 
		SX=D0, SY=D1, SZ=D2, 
		W=D3 
	};
	matrix() { Identity(); }
	void Identity()
	{
		entry[1] = entry[2] = entry[TX] = entry[4] = entry[6] = entry[TY] =
		entry[8] = entry[9] = entry[TZ] = entry[12] = entry[13] = entry[14] = 0;
		entry[D0] = entry[D1] = entry[D2] = entry[W] = 1;
	}
	void Rotate( vector3 a_Pos, real a_RX, real a_RY, real a_RZ )
	{
		matrix t;
		t.RotateX( a_RZ );
		RotateY( a_RY );
		Concatenate( t );
		t.RotateZ( a_RX );
		Concatenate( t );
		Translate( a_Pos );
	}
	void RotateX( real a_RX )
	{
		real sx = (real)sin( a_RX * PI / 180 );
		real cx = (real)cos( a_RX * PI / 180 );
		Identity();
		entry[5] = cx, entry[6] = sx, entry[9] = -sx, entry[10] = cx;
	}
	void RotateY( real a_RY )
	{
		real sy = (real)sin( a_RY * PI / 180 );
		real cy = (real)cos( a_RY * PI / 180 );
		Identity ();
		entry[0] = cy, entry[2] = -sy, entry[8] = sy, entry[10] = cy;
	}
	void RotateZ( real a_RZ )
	{
		real sz = (real)sin( a_RZ * PI / 180 );
		real cz = (real)cos( a_RZ * PI / 180 );
		Identity ();
		entry[0] = cz, entry[1] = sz, entry[4] = -sz, entry[5] = cz;
	}
	void Translate( vector3 a_Pos ) { entry[TX] += a_Pos.x; entry[TY] += a_Pos.y; entry[TZ] += a_Pos.z; }
	void Concatenate( matrix& m2 )
	{
		matrix res;
		for ( int c = 0; c < 4; c++ ) for ( int r = 0; r < 4; r++ )
			res.entry[r * 4 + c] = entry[r * 4] * m2.entry[c] +
				  				  entry[r * 4 + 1] * m2.entry[c + 4] +
								  entry[r * 4 + 2] * m2.entry[c + 8] +
								  entry[r * 4 + 3] * m2.entry[c + 12];
		for ( int c = 0; c < 16; c++ ) entry[c] = res.entry[c];
	}
	vector3 Transform( vector3& v )
	{
		real x  = entry[0] * v.x + entry[1] * v.y + entry[2] * v.z + entry[3];
		real y  = entry[4] * v.x + entry[5] * v.y + entry[6] * v.z + entry[7];
		real z  = entry[8] * v.x + entry[9] * v.y + entry[10] * v.z + entry[11];
		return vector3( x, y, z );
	}
	void Invert()
	{
		matrix t;
		real tx = -entry[3], ty = -entry[7], tz = -entry[11];
		for ( int h = 0; h < 3; h++ ) for ( int v = 0; v < 3; v++ ) t.entry[h + v * 4] = entry[v + h * 4];
		for ( int i = 0; i < 11; i++ ) entry[i] = t.entry[i];
		entry[3] = tx * entry[0] + ty * entry[1] + tz * entry[2];
		entry[7] = tx * entry[4] + ty * entry[5] + tz * entry[6];
		entry[11] = tx * entry[8] + ty * entry[9] + tz * entry[10];
	}
	real entry[16];
};

class plane
{
public:
	plane() : N( 0, 0, 0 ), D( 0 ) {};
	plane( vector3 a_Normal, real a_D ) : N( a_Normal ), D( a_D ) {};
	union
	{
		struct
		{
			vector3 N;
			real D;
		};
		real entry[4];
	};
};

class aabb
{
public:
	aabb() : m_Pos( vector3( 0, 0, 0 ) ), m_Size( vector3( 0, 0, 0 ) ) {};
	aabb( vector3& a_Pos, vector3& a_Size ) : m_Pos( a_Pos ), m_Size( a_Size ) {};
	vector3& GetPos() { return m_Pos; }
	vector3& GetSize() { return m_Size; }
	bool Intersect( aabb& b2 )
	{
		vector3 v1 = b2.GetPos(), v2 = b2.GetPos() + b2.GetSize();
		vector3 v3 = m_Pos, v4 = m_Pos + m_Size;
		return ((v4.x >= v1.x) && (v3.x <= v2.x) && // x-axis overlap
				(v4.y >= v1.y) && (v3.y <= v2.y) && // y-axis overlap
				(v4.z >= v1.z) && (v3.z <= v2.z));   // z-axis overlap
	}
	bool Contains( vector3 a_Pos )
	{
		vector3 v1 = m_Pos, v2 = m_Pos + m_Size;
		return ((a_Pos.x >= v1.x) && (a_Pos.x <= v2.x) &&
				(a_Pos.y >= v1.y) && (a_Pos.y <= v2.y) &&
				(a_Pos.z >= v1.z) && (a_Pos.z <= v2.z));
	}
	real w() { return m_Size.x; }
	real h() { return m_Size.y; }
	real d() { return m_Size.z; }
	real x() { return m_Pos.x; }
	real y() { return m_Pos.y; }
	real z() { return m_Pos.z; }
private:
	vector3 m_Pos, m_Size;
};

typedef vector3 Color;

}; // namespace Raytracer

#endif