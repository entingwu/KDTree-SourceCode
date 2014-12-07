#include "common.h"
#include "string.h"
#include "scene.h"
#include "raytracer.h"
#include "stdio.h"
#include "memory.h"
#include "surface.h"
#include "loadMesh.h"
namespace Raytracer {

MManager* KdTree::s_MManager = 0;
Material::Material() :
	m_Color( Color( 0.2f, 0.2f, 0.2f ) ),
	m_Refl( 0 ), m_Diff( 0.2f ), m_Spec( 0.8f ), 
	m_RIndex( 1.5f ), m_DRefl( 0 ), 
	m_UScale( 1.0f ), m_VScale( 1.0f )
{
}

void Material::SetUVScale( real a_UScale, real a_VScale )
{ 
	m_UScale = a_UScale; 
	m_VScale = a_VScale; 
	m_RUScale = 1.0f / a_UScale;
	m_RVScale = 1.0f / a_VScale;
}

void Material::SetParameters( real a_Refl, real a_Refr, Color& a_Col, real a_Diff, real a_Spec )
{
	m_Refl = a_Refl;
	m_Refr = a_Refr;
	m_Color = a_Col;
	m_Diff = a_Diff;
	m_Spec = a_Spec;
}

// -----------------------------------------------------------
// Primitive methods
// -----------------------------------------------------------

Primitive::Primitive( int a_Type, vector3& a_Centre, real a_Radius )
{
	m_Centre = a_Centre;
	m_SqRadius = a_Radius * a_Radius;
	m_Radius = a_Radius;
	m_RRadius = 1.0f / a_Radius;
	m_Type = a_Type;
	m_Material = new Material();
	// set vectors for texture mapping
	m_Vn = vector3( 0, 1, 0 );
	m_Ve = vector3( 1, 0, 0 );
	m_Vc = m_Vn.Cross( m_Ve );
}

Primitive::Primitive( int a_Type, Vertex* a_V1, Vertex* a_V2, Vertex* a_V3 )
{
	m_Type = a_Type;
	m_Material = 0;
	m_Vertex[0] = a_V1;
	m_Vertex[1] = a_V2;
	m_Vertex[2] = a_V3;
	// init precomp
	vector3 A = m_Vertex[0]->GetPos();
	vector3 B = m_Vertex[1]->GetPos();
	vector3 C = m_Vertex[2]->GetPos();
	vector3 c = B - A;
	vector3 b = C - A;
	m_N = b.Cross( c );
	int u, v;
	if (_fabs( m_N.x ) > _fabs( m_N.y))
	{
		if (_fabs( m_N.x ) > _fabs( m_N.z )) k = 0; else k = 2;
	}
	else
	{
		if (_fabs( m_N.y ) > _fabs( m_N.z )) k = 1; else k = 2;
	}
	u = (k + 1) % 3;
	v = (k + 2) % 3;
	// precomp
	real krec = 1.0f / m_N.entry[k];
	nu = m_N.entry[u] * krec;
	nv = m_N.entry[v] * krec;
	nd = m_N.Dot( A ) * krec;
	// first line equation
	real reci = 1.0f / (b.entry[u] * c.entry[v] - b.entry[v] * c.entry[u]);
	bnu = b.entry[u] * reci;
	bnv = -b.entry[v] * reci;
	// second line equation
	cnu = c.entry[v] * reci;
	cnv = -c.entry[u] * reci;
	// finalize normal
	m_N.Normalize();
	m_Vertex[0]->SetNormal( m_N );
	m_Vertex[1]->SetNormal( m_N );
	m_Vertex[2]->SetNormal( m_N );
}

Primitive::~Primitive()
{
	if (m_Type == SPHERE) delete m_Material;
}

unsigned int modulo[] = { 0, 1, 2, 0, 1 };
int Primitive::Intersect( Ray& a_Ray, real& a_Dist )
{
	if (m_Type == SPHERE)
	{
		vector3 v = a_Ray.GetOrigin() - m_Centre;
		real b = -DOT( v, a_Ray.GetDirection() );
		real det = (b * b) - DOT( v, v ) + m_SqRadius;
		int retval = MISS;
		if (det > 0)
		{
			det = _sqrt( det );
			real i1 = b - det;
			real i2 = b + det;
			if (i2 > 0)
			{
				if (i1 < 0) 
				{
					if (i2 < a_Dist) 
					{
						a_Dist = i2;
						retval = INPRIM;
					}
				}
				else
				{
					if (i1 < a_Dist)
					{
						a_Dist = i1;
						retval = HIT;
					}
				}
			}
		}
		return retval;
	}
	else
	{
		#define ku modulo[k + 1]
		#define kv modulo[k + 2]
		vector3 O = a_Ray.GetOrigin(), D = a_Ray.GetDirection(), A = m_Vertex[0]->GetPos();
		const real lnd = 1.0f / (D.entry[k] + nu * D.entry[ku] + nv * D.entry[kv]);
		const real t = (nd - O.entry[k] - nu * O.entry[ku] - nv * O.entry[kv]) * lnd;
		if (!(a_Dist > t && t > 0)) return MISS;
		real hu = O.entry[ku] + t * D.entry[ku] - A.entry[ku];
		real hv = O.entry[kv] + t * D.entry[kv] - A.entry[kv];
		real beta = m_U = hv * bnu + hu * bnv;
		if (beta < 0) return MISS;
		real gamma = m_V = hu * cnu + hv * cnv;
		if (gamma < 0) return MISS;
		if ((m_U + m_V) > 1) return MISS;
		a_Dist = t;
		return (DOT( D, m_N ) > 0)?INPRIM:HIT;
	}
}

vector3 Primitive::GetNormal( vector3& a_Pos ) 
{ 
	if (m_Type == SPHERE) 
	{
		return (a_Pos - m_Centre) * m_RRadius; 
	}
	else 
	{
		vector3 N1 = m_Vertex[0]->GetNormal();
		vector3 N2 = m_Vertex[1]->GetNormal();
		vector3 N3 = m_Vertex[2]->GetNormal();
		vector3 N = N1 + m_U * (N2 - N1) + m_V * (N3 - N1);
		NORMALIZE( N );
		return N;
	}
}
	
Color Primitive::GetColor( vector3& a_Pos )
{
	Color retval;
	retval = m_Material->GetColor();
	return retval;
}

#define FINDMINMAX( x0, x1, x2, min, max ) \
  min = max = x0; if(x1<min) min=x1; if(x1>max) max=x1; if(x2<min) min=x2; if(x2>max) max=x2;
// X-tests
#define AXISTEST_X01( a, b, fa, fb )											\
	p0 = a * v0.entry[1] - b * v0.entry[2], p2 = a * v2.entry[1] - b * v2.entry[2]; \
    if (p0 < p2) { min = p0; max = p2;} else { min = p2; max = p0; }			\
	rad = fa * a_BoxHalfsize.entry[1] + fb * a_BoxHalfsize.entry[2];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_X2( a, b, fa, fb )												\
	p0 = a * v0.entry[1] - b * v0.entry[2], p1 = a * v1.entry[1] - b * v1.entry[2];	\
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0;}			\
	rad = fa * a_BoxHalfsize.entry[1] + fb * a_BoxHalfsize.entry[2];				\
	if(min>rad || max<-rad) return 0;
// Y-tests
#define AXISTEST_Y02( a, b, fa, fb )											\
	p0 = -a * v0.entry[0] + b * v0.entry[2], p2 = -a * v2.entry[0] + b * v2.entry[2]; \
    if(p0 < p2) { min = p0; max = p2; } else { min = p2; max = p0; }			\
	rad = fa * a_BoxHalfsize.entry[0] + fb * a_BoxHalfsize.entry[2];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_Y1( a, b, fa, fb )												\
	p0 = -a * v0.entry[0] + b * v0.entry[2], p1 = -a * v1.entry[0] + b * v1.entry[2]; \
    if (p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }			\
	rad = fa * a_BoxHalfsize.entry[0] + fb * a_BoxHalfsize.entry[2];				\
	if (min > rad || max < -rad) return 0;
// Z-tests
#define AXISTEST_Z12( a, b, fa, fb )											\
	p1 = a * v1.entry[0] - b * v1.entry[1], p2 = a * v2.entry[0] - b * v2.entry[1]; \
    if(p2 < p1) { min = p2; max = p1; } else { min = p1; max = p2; }			\
	rad = fa * a_BoxHalfsize.entry[0] + fb * a_BoxHalfsize.entry[1];				\
	if (min > rad || max < -rad) return 0;
#define AXISTEST_Z0( a, b, fa, fb )												\
	p0 = a * v0.entry[0] - b * v0.entry[1], p1 = a * v1.entry[0] - b * v1.entry[1];	\
    if(p0 < p1) { min = p0; max = p1; } else { min = p1; max = p0; }			\
	rad = fa * a_BoxHalfsize.entry[0] + fb * a_BoxHalfsize.entry[1];				\
	if (min > rad || max < -rad) return 0;

bool Primitive::PlaneBoxOverlap( vector3& a_Normal, vector3& a_Vert, vector3& a_MaxBox )
{
	vector3 vmin, vmax;
	for( int q = 0; q < 3; q++ )
	{
		float v = a_Vert.entry[q];
		if (a_Normal.entry[q] > 0.0f)
		{
			vmin.entry[q] = -a_MaxBox.entry[q] - v;
			vmax.entry[q] =  a_MaxBox.entry[q] - v;
		}
		else
		{
			vmin.entry[q] =  a_MaxBox.entry[q] - v;
			vmax.entry[q] = -a_MaxBox.entry[q] - v;
		}
	}
	if (DOT( a_Normal, vmin) > 0.0f) return false;
	if (DOT( a_Normal, vmax) >= 0.0f) return true;
	return false;
}

bool Primitive::IntersectTriBox( vector3& a_BoxCentre, vector3& a_BoxHalfsize, vector3& a_V0, vector3& a_V1, vector3& a_V2 )
{
	vector3 v0, v1, v2, normal, e0, e1, e2;
	float min, max, p0, p1, p2, rad, fex, fey, fez;
	v0 = a_V0 - a_BoxCentre;
	v1 = a_V1 - a_BoxCentre;
	v2 = a_V2 - a_BoxCentre;
	e0 = v1 - v0, e1 = v2 - v1, e2 = v0 - v2;
	fex = fabsf( e0.entry[0] );
	fey = fabsf( e0.entry[1] );
	fez = fabsf( e0.entry[2] );
	AXISTEST_X01( e0.entry[2], e0.entry[1], fez, fey );
	AXISTEST_Y02( e0.entry[2], e0.entry[0], fez, fex );
	AXISTEST_Z12( e0.entry[1], e0.entry[0], fey, fex );
	fex = fabsf( e1.entry[0] );
	fey = fabsf( e1.entry[1] );
	fez = fabsf( e1.entry[2] );
	AXISTEST_X01( e1.entry[2], e1.entry[1], fez, fey );
	AXISTEST_Y02( e1.entry[2], e1.entry[0], fez, fex );
	AXISTEST_Z0 ( e1.entry[1], e1.entry[0], fey, fex );
	fex = fabsf( e2.entry[0] );
	fey = fabsf( e2.entry[1] );
	fez = fabsf( e2.entry[2] );
	AXISTEST_X2 ( e2.entry[2], e2.entry[1], fez, fey );
	AXISTEST_Y1 ( e2.entry[2], e2.entry[0], fez, fex );
	AXISTEST_Z12( e2.entry[1], e2.entry[0], fey, fex );
	FINDMINMAX( v0.entry[0], v1.entry[0], v2.entry[0], min, max );
	if (min > a_BoxHalfsize.entry[0] || max < -a_BoxHalfsize.entry[0]) return false;
	FINDMINMAX( v0.entry[1], v1.entry[1], v2.entry[1], min, max );
	if (min > a_BoxHalfsize.entry[1] || max < -a_BoxHalfsize.entry[1]) return false;
	FINDMINMAX( v0.entry[2], v1.entry[2], v2.entry[2], min, max );
	if (min > a_BoxHalfsize.entry[2] || max < -a_BoxHalfsize.entry[2]) return false;
	normal = e0.Cross( e1 );
	if (!PlaneBoxOverlap( normal, v0, a_BoxHalfsize )) return false;
	return true;
}

bool Primitive::IntersectSphereBox( vector3& a_Centre, aabb& a_Box )
{
	float dmin = 0;
	vector3 spos = a_Centre;
	vector3 bpos = a_Box.GetPos();
	vector3 bsize = a_Box.GetSize();
	for ( int i = 0; i < 3; i++ )
	{
		if (spos.entry[i] < bpos.entry[i]) 
		{
			dmin = dmin + (spos.entry[i] - bpos.entry[i]) * (spos.entry[i] - bpos.entry[i]);
		}
		else if (spos.entry[i] > (bpos.entry[i] + bsize.entry[i])) 
		{
			dmin = dmin + (spos.entry[i] - (bpos.entry[i] + bsize.entry[i])) * (spos.entry[i] - (bpos.entry[i] + bsize.entry[i]));
		}
	}
	return (dmin <= m_SqRadius);
}

bool Primitive::IntersectBox( aabb& a_Box )
{
	if (m_Type == SPHERE)
	{
		return IntersectSphereBox( m_Centre, a_Box );
	}
	else
	{
		return IntersectTriBox( a_Box.GetPos() + a_Box.GetSize() * 0.5f, a_Box.GetSize() * 0.5f, 
								m_Vertex[0]->GetPos(), m_Vertex[1]->GetPos(), m_Vertex[2]->GetPos() );
	}
}

void Primitive::CalculateRange( real& a_Pos1, real& a_Pos2, int a_Axis )
{
	if (m_Type == SPHERE)
	{
		a_Pos1 = m_Centre.entry[a_Axis] - m_Radius;
		a_Pos2 = m_Centre.entry[a_Axis] + m_Radius;
	}
	else
	{
		vector3 pos1 = m_Vertex[0]->GetPos();
		a_Pos1 = pos1.entry[a_Axis], a_Pos2 = pos1.entry[a_Axis];
		for ( int i = 1; i < 3; i++ )
		{
			vector3 pos = m_Vertex[i]->GetPos();
			if (pos.entry[a_Axis] < a_Pos1) a_Pos1 = pos.entry[a_Axis];
			if (pos.entry[a_Axis] > a_Pos2) a_Pos2 = pos.entry[a_Axis];
		}
	}
}


Light::Light( int a_Type, vector3& a_P1, vector3& a_P2, vector3& a_P3, Color& a_Color )
{
	m_Type = a_Type;
	m_Color = a_Color;
	m_Grid = new vector3[16];
	m_Grid[ 0] = vector3( 1, 2, 0 );
	m_Grid[ 1] = vector3( 3, 3, 0 );
	m_Grid[ 2] = vector3( 2, 0, 0 );
	m_Grid[ 3] = vector3( 0, 1, 0 );
	m_Grid[ 4] = vector3( 2, 3, 0 );
	m_Grid[ 5] = vector3( 0, 3, 0 );
	m_Grid[ 6] = vector3( 0, 0, 0 );
	m_Grid[ 7] = vector3( 2, 2, 0 );
	m_Grid[ 8] = vector3( 3, 1, 0 );
	m_Grid[ 9] = vector3( 1, 3, 0 );
	m_Grid[10] = vector3( 1, 0, 0 );
	m_Grid[11] = vector3( 3, 2, 0 );
	m_Grid[12] = vector3( 2, 1, 0 );
	m_Grid[13] = vector3( 3, 0, 0 );
	m_Grid[14] = vector3( 1, 1, 0 );
	m_Grid[15] = vector3( 0, 2, 0 );
	m_CellX = (a_P2 - a_P1) * 0.25f;
	m_CellY = (a_P3 - a_P1) * 0.25f;
	for ( int i = 0; i < 16; i++ )
		m_Grid[i] = m_Grid[i].entry[0] * m_CellX + m_Grid[i].entry[1] * m_CellY + a_P1;
	m_Pos = a_P1 + 2 * m_CellX + 2 * m_CellY;
}

// -----------------------------------------------------------
// Scene class implementation
// -----------------------------------------------------------

Scene::Scene() :
	m_Primitives( 0 ), 
	m_Primitive( 0 ), 
	m_Extends( vector3( 0, 0, 0 ), vector3( 0, 0, 0 ) ), 
	m_State( 0 )
{
}

Scene::~Scene()
{
	delete m_Primitive;
}



unsigned char* temp = new unsigned char[8];
unsigned char* aligned = (unsigned char*)(((unsigned long)temp + 4) & 0xFFFFFFFC);
inline float getf( char* addr )
{
	memcpy( aligned, addr, 4 );
	return *(float*)aligned;
}
inline unsigned short getw( char* addr )
{
	memcpy( aligned, addr, 2 );
	return *(unsigned short*)aligned;
}
inline unsigned long getd( char* addr )
{
	memcpy( aligned, addr, 4 );
	return *(unsigned long*)aligned;
}

long Scene::EatChunk( char* buffer )
{
	short chunkid = getw( buffer );
	long chunklength = getd( buffer + 2 );
	int j, i = 6, cp = 0, tcoords = 0;
	switch (chunkid)
	{
	case 0x4D4D:
		while ((getw( buffer + i) != 0x3D3D) && (getw( buffer + i) != 0xB000)) i += 2;
		break;
	case 0x4000:
		while (*(buffer + (i++)) != 0);
		break;
	case 0x4110:
		m_NrVerts = getw( buffer + i );
		delete m_Verts;
		m_Verts = new float[m_NrVerts * 3];
		i += 2;
		for ( j = 0; j < m_NrVerts; j++ )
		{
			m_Verts[j * 3] = getf( buffer + i );
			m_Verts[j * 3 + 1] = getf( buffer + i + 4 );
			m_Verts[j * 3 + 2] = getf( buffer + i + 8 );
			i += 12;
		}
		break;
	case 0x4120:
		m_NrFaces = getw( buffer + i );
		delete m_Faces;
		m_Faces = new unsigned short[m_NrFaces * 3];
		i += 2;
		for ( j = 0; j < m_NrFaces; j++ )
		{
			m_Faces[j * 3] = getw( buffer + i );
			m_Faces[j * 3 + 1] = getw( buffer + i + 2 );
			m_Faces[j * 3 + 2] = getw( buffer + i + 4 );
			i += 8;
		}
	case 0x4140:
		tcoords = getw( buffer + i );
		delete m_TCoords;
		m_TCoords = new float[tcoords * 2];
		i += 2;
		for ( j = 0; j < tcoords; j++ )
		{
			m_TCoords[j * 2] = getf( buffer + i );
			m_TCoords[j * 2 + 1] = getf( buffer + i + 4 );
			i += 8;
		}
	case 0x3D3D:
	case 0x4100:
		break;
	default:
		i = chunklength;
	break;
	}
	while (i < chunklength) i += EatChunk( buffer + i );
	return chunklength;
}
bool Scene::InitScene( Surface* a_MsgSurf )
{
	Material* mat;
	int x;
	vector3 p1, p2;
	switch (m_State)
	{
	case 0:
		//a_MsgSurf->Print( "constructing geometry", 2, 2, 0xffffffff );
		break;
	case 1:
		m_Primitive = new Primitive*[100000];
		m_Primitives = 0;
		m_Light = new Light*[MAXLIGHTS];
		m_Lights = 0;
	
		//m_Light[m_Lights++] = new Light( Light::POINT, vector3( 0, 5, 5 ), Color( 1.0f, 1.0f, 1.0f ) );
		m_Light[m_Lights++] = new Light( Light::POINT, vector3( 0, 0, 0 ), Color( 1.0f, 1.0f, 1.0f ) );
		//m_Light[m_Lights++] = new Light( Light::POINT, vector3( -3, 5, 1 ), Color( 1.0f, 1.0f, 1.0f ) );
		mat = new Material();
		mat->SetParameters( 0.9f, 0, Color( 1.0f, 1.0f, 1.0f ), 0.5f, 0.5f );
	
		load_mesh("sibenik.obj");

		for(int i=0;i<60000;i++)
	  {
		vector3 v1(gPositions[gTriangles[i].indices[0]].x,gPositions[gTriangles[i].indices[0]].y,gPositions[gTriangles[i].indices[0]].z);
		vector3 v2(gPositions[gTriangles[i].indices[1]].x,gPositions[gTriangles[i].indices[1]].y,gPositions[gTriangles[i].indices[1]].z);
		vector3 v3(gPositions[gTriangles[i].indices[2]].x,gPositions[gTriangles[i].indices[2]].y,gPositions[gTriangles[i].indices[2]].z);
	
		Vertex* V1,*V2,*V3;
		V1=new Vertex(v1,0.0f,0.0f);
		V2=new Vertex(v2,0.0f,0.0f);
		V3=new Vertex(v3,0.0f,0.0f);
		V1->SetUV(V1->GetPos().entry[0]/6.00f,V1->GetPos().entry[1]);
		V2->SetUV(V2->GetPos().entry[0]/6.00f,V2->GetPos().entry[1]);
		V3->SetUV(V3->GetPos().entry[0]/6.00f,V3->GetPos().entry[1]);

		m_Primitive[m_Primitives] = new Primitive(Primitive::TRIANGLE, V1,V3,V2);
		//m_Primitive[m_Primitives] = new Primitive(Primitive::TRIANGLE, V1,V2,V3);
		m_Primitive[m_Primitives]->SetMaterial(mat);
	

		m_Primitives++;

	    }


		break;
	case 2:
		a_MsgSurf->Print( "K-D Tree Generating", 2, 10, 0xffffffff );
		break;
	case 3:
		// build the kd-tree
		p1 = vector3( -100, -100, -100 ), p2 = vector3( 100, 100, 100 );
		m_Extends = aabb( p1, p2 - p1 );
		m_KdTree = new KdTree();
		m_KdTree->Build( this );
		break;
	default:
		return true;
	};
	m_State++;
	return false;
}

KdTree::KdTree()
{
	m_Root = new KdTreeNode();
}

void KdTree::Build( Scene* a_Scene )
{
	for ( int p = 0; p < a_Scene->GetNrPrimitives(); p++ )
		m_Root->Add( a_Scene->GetPrimitive( p ) );
	
	int prims = a_Scene->GetNrPrimitives();
	aabb sbox = a_Scene->GetExtends();
	m_SPool = new SplitList[prims * 2 + 8];
	int i;
	for ( i = 0; i < (prims * 2 + 6); i++ ) m_SPool[i].next = &m_SPool[i + 1];
	m_SPool[i].next = 0;
	m_SList = 0;
	Subdivide( m_Root, sbox, 0, prims );
}

void KdTree::InsertSplitPos( real a_SplitPos )
{
	// insert a split position candidate in the list if unique
	SplitList* entry = m_SPool;
	m_SPool = m_SPool->next;
	entry->next = 0;
	entry->splitpos = a_SplitPos;
	entry->n1count = 0;
	entry->n2count = 0;
	if (!m_SList) m_SList = entry; else
	{
		if (a_SplitPos < m_SList->splitpos)
		{
			entry->next = m_SList;
			m_SList = entry;
		}
		else if (a_SplitPos == m_SList->splitpos)
		{
			entry->next = m_SPool; // redundant; recycle
			m_SPool = entry;
		}
		else
		{
			SplitList* list = m_SList;
			while ((list->next) && (a_SplitPos >= list->next->splitpos)) 
			{
				if (a_SplitPos == list->next->splitpos)
				{
					entry->next = m_SPool; // redundant; recycle
					m_SPool = entry;
					return;
				}
				list = list->next;
			}
			entry->next = list->next;
			list->next = entry;
		}
	}
}

void KdTree::Subdivide( KdTreeNode* a_Node, aabb& a_Box, int a_Depth, int a_Prims )
{
	// recycle used split list nodes
	if (m_SList)
	{
		SplitList* list = m_SList;
		while (list->next) list = list->next;
		list->next = m_SPool;
		m_SPool = m_SList, m_SList = 0;
	}
	// determine split axis
	vector3 s = a_Box.GetSize();
	if ((s.x >= s.y) && (s.x >= s.z)) a_Node->SetAxis( 0 );
	else if ((s.y >= s.x) && (s.y >= s.z)) a_Node->SetAxis( 1 );
	int axis = a_Node->GetAxis();
	// make a list of the split position candidates
	ObjectList* l = a_Node->GetList();
	real p1, p2;
	real pos1 = a_Box.GetPos().entry[axis];
	real pos2 = a_Box.GetPos().entry[axis] + a_Box.GetSize().entry[axis];
	bool* pright = new bool[a_Prims];
	float* eleft = new float[a_Prims], *eright = new float[a_Prims];
	Primitive** parray = new Primitive*[a_Prims];
	int aidx = 0;
	while (l)
	{
		Primitive* p = parray[aidx] = l->GetPrimitive();
		pright[aidx] = true;
		p->CalculateRange( eleft[aidx], eright[aidx], axis );
		aidx++;
		if (p->GetType() == Primitive::SPHERE)
		{
			p1 = p->GetCentre().entry[axis] - p->GetRadius();
			p2 = p->GetCentre().entry[axis] + p->GetRadius();
			if ((p1 >= pos1) && (p1 <= pos2)) InsertSplitPos( p1 );
			if ((p2 >= pos1) && (p2 <= pos2)) InsertSplitPos( p2 );
		}
		else
		{
			for ( int i = 0; i < 3; i++ )
			{
				p1 = p->GetVertex( i )->GetPos().entry[axis];
				if ((p1 >= pos1) && (p1 <= pos2)) InsertSplitPos( p1 );
			}
		}
		l = l->GetNext();
	}
	// determine n1count / n2count for each split position
	aabb b1, b2, b3 = a_Box, b4 = a_Box;
	SplitList* splist = m_SList;
	float b3p1 = b3.GetPos().entry[axis];
	float b4p2 = b4.GetPos().entry[axis] + b4.GetSize().entry[axis];
	while (splist)
	{
		b4.GetPos().entry[axis] = splist->splitpos;
		b4.GetSize().entry[axis] = pos2 - splist->splitpos;
		b3.GetSize().entry[axis] = splist->splitpos - pos1;
		float b3p2 = b3.GetPos().entry[axis] + b3.GetSize().entry[axis];
		float b4p1 = b4.GetPos().entry[axis];
		for ( int i = 0; i < a_Prims; i++ ) if (pright[i])
		{
			Primitive* p = parray[i];
			if ((eleft[i] <= b3p2) && (eright[i] >= b3p1))
				if (p->IntersectBox( b3 )) splist->n1count++;
			if ((eleft[i] <= b4p2) && (eright[i] >= b4p1))
				if (p->IntersectBox( b4 )) splist->n2count++; else pright[i] = false;
		}
		else splist->n1count++;
		splist = splist->next;
	}
	delete pright;
	// calculate surface area for current node
	real SAV = 0.5f / (a_Box.w() * a_Box.d() + a_Box.w() * a_Box.h() + a_Box.d() * a_Box.h());
	// calculate cost for not splitting
	real Cleaf = a_Prims * 1.0f;
	// determine optimal split plane position
	splist = m_SList;
	real lowcost = 50000;
	real bestpos = 0;
	while (splist)
	{
		// calculate child node extends
		b4.GetPos().entry[axis] = splist->splitpos;
		b4.GetSize().entry[axis] = pos2 - splist->splitpos;
		b3.GetSize().entry[axis] = splist->splitpos - pos1;
		// calculate child node cost
		real SA1 = 2 * (b3.w() * b3.d() + b3.w() * b3.h() + b3.d() * b3.h());
		real SA2 = 2 * (b4.w() * b4.d() + b4.w() * b4.h() + b4.d() * b4.h());
		real splitcost = 0.3f + 1.0f * (SA1 * SAV * splist->n1count + SA2 * SAV * splist->n2count);
		// update best cost tracking variables
		if (splitcost < lowcost)
		{
			lowcost = splitcost;
			bestpos = splist->splitpos;
			b1 = b3, b2 = b4;
		}
		splist = splist->next;
	}
	if (lowcost > Cleaf) return;
	a_Node->SetSplitPos( bestpos );
	// construct child nodes
	KdTreeNode* left = s_MManager->NewKdTreeNodePair();
	int n1count = 0, n2count = 0, total = 0;
	// assign primitives to both sides
	float b1p1 = b1.GetPos().entry[axis];
	float b2p2 = b2.GetPos().entry[axis] + b2.GetSize().entry[axis];
	float b1p2 = b1.GetPos().entry[axis] + b1.GetSize().entry[axis];
	float b2p1 = b2.GetPos().entry[axis];
	for ( int i = 0; i < a_Prims; i++ )
	{
		Primitive* p = parray[i];
		total++;
		if ((eleft[i] <= b1p2) && (eright[i] >= b1p1)) if (p->IntersectBox( b1 )) 
		{
			left->Add( p );
			n1count++;
		}
		if ((eleft[i] <= b2p2) && (eright[i] >= b2p1)) if (p->IntersectBox( b2 )) 
		{
			(left + 1)->Add( p );
			n2count++;
		}
	}
	delete eleft;
	delete eright;
	delete parray;
	s_MManager->FreeObjectList( a_Node->GetList() );
	a_Node->SetLeft( left );
	a_Node->SetLeaf( false );
	if (a_Depth < MAXTREEDEPTH)
	{
		if (n1count > 2) Subdivide( left, b1, a_Depth + 1, n1count );
		if (n2count > 2) Subdivide( left + 1, b2, a_Depth + 1, n2count );
	}
}
void KdTreeNode::Add( Primitive* a_Prim )
{
	ObjectList* lnode = KdTree::s_MManager->NewObjectList();
	lnode->SetPrimitive( a_Prim );
	lnode->SetNext( GetList() );
	SetList( lnode );
}

}; // namespace Raytracer