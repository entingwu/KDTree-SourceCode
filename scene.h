
#ifndef I_SCENE_H
#define I_SCENE_H

#include "raytracer.h"

namespace Raytracer {

// Intersection method return values
#define HIT		 1		// Ray hit primitive
#define MISS	 0		// Ray missed primitive
#define INPRIM	-1		// Ray started inside primitive

#define MAXLIGHTS	10

class Material
{
public:
	Material();
	void SetColor( Color& a_Color ) { m_Color = a_Color; }
	Color GetColor() { return m_Color; }
	void SetDiffuse( real a_Diff ) { m_Diff = a_Diff; }
	void SetSpecular( real a_Spec ) { m_Spec = a_Spec; }
	void SetReflection( real a_Refl ) { m_Refl = a_Refl; }
	void SetRefraction( real a_Refr ) { m_Refr = a_Refr; }
	void SetParameters( real a_Refl, real a_Refr, Color& a_Col, real a_Diff, real a_Spec );
	real GetSpecular() { return m_Spec; }
	real GetDiffuse() { return m_Diff; }
	real GetReflection() { return m_Refl; }
	real GetRefraction() { return m_Refr; }
	void SetRefrIndex( real a_Refr ) { m_RIndex = a_Refr; }
	real GetRefrIndex() { return m_RIndex; }
	void SetDiffuseRefl( real a_DRefl ) { m_DRefl = a_DRefl; }
	real GetDiffuseRefl() { return m_DRefl; }
	//void SetTexture( Texture* a_Texture ) { m_Texture = a_Texture; }
	//Texture* GetTexture() { return m_Texture; }
	void SetUVScale( real a_UScale, real a_VScale );
	real GetUScale() { return m_UScale; }
	real GetVScale() { return m_VScale; }
	real GetUScaleReci() { return m_RUScale; }
	real GetVScaleReci() { return m_RVScale; }
private:
	Color m_Color;
	real m_Refl, m_Refr;
	real m_Diff, m_Spec;
	real m_DRefl;
	real m_RIndex;
	//Texture* m_Texture;
	real m_UScale, m_VScale, m_RUScale, m_RVScale;
};

// -----------------------------------------------------------
// Primitive class definition
// -----------------------------------------------------------

class Vertex
{
public:
	Vertex() {};
	Vertex( vector3 a_Pos, real a_U, real a_V ) : m_Pos( a_Pos ), m_U( a_U ), m_V( a_V ) {};
	real GetU() { return m_U; }
	real GetV() { return m_V; }
	vector3& GetNormal() { return m_Normal; }
	vector3& GetPos() { return m_Pos; }
	void SetUV( real a_U, real a_V ) { m_U = a_U; m_V = a_V; }
	void SetPos( vector3& a_Pos ) { m_Pos = a_Pos; }
	void SetNormal( vector3& a_N ) { m_Normal = a_N; }
private:
	vector3 m_Pos;
	vector3 m_Normal;
	real m_U, m_V;
};

class Light
{
public:
	enum
	{
		POINT = 1,
		AREA
	};
	Light( int a_Type, vector3& a_Pos, Color& a_Color ) : m_Type( a_Type ), m_Pos( a_Pos ), m_Color( a_Color ), m_Grid( 0 ) {};
	Light( int a_Type, vector3& a_P1, vector3& a_P2, vector3& a_P3, Color& a_Color );
	vector3& GetPos() { return m_Pos; }
	vector3& GetCellX() { return m_CellX; }
	vector3& GetCellY() { return m_CellY; }
	vector3& GetGrid( int a_Idx ) { return m_Grid[a_Idx]; }
	Color& GetColor() { return m_Color; }
	int GetType() { return m_Type; }
private:
	vector3 m_Pos, m_CellX, m_CellY;
	Color m_Color;
	int m_Type;
	vector3* m_Grid;
};

class Primitive
{
public:
	enum
	{
		SPHERE = 1,
		TRIANGLE
	};
	Primitive() {};
	Primitive( int a_Type, vector3& a_Centre, real a_Radius );
	Primitive( int a_Type, Vertex* a_V1, Vertex* a_V2, Vertex* a_V3 );
	~Primitive();
	Material* GetMaterial() { return m_Material; }
	void SetMaterial( Material* a_Mat ) { m_Material = a_Mat; }
	int GetType() { return m_Type; }
	int Intersect( Ray& a_Ray, real& a_Dist );
	bool IntersectBox( aabb& a_Box );
	void CalculateRange( real& a_Pos1, real& a_Pos2, int a_Axis );
	vector3 GetNormal( vector3& a_Pos );
	Color GetColor( vector3& );
	// triangle-box intersection stuff
	bool PlaneBoxOverlap( vector3& a_Normal, vector3& a_Vert, vector3& a_MaxBox );
	bool IntersectTriBox( vector3& a_BoxCentre, vector3& a_BoxHalfsize, vector3& a_V0, vector3& a_V1, vector3& a_V2 );
	bool IntersectSphereBox( vector3& a_Centre, aabb& a_Box );
	// sphere primitive methods
	vector3& GetCentre() { return m_Centre; }
	real GetSqRadius() { return m_SqRadius; }
	real GetRadius() { return m_Radius; }
	// triangle primitive methods
	vector3 GetNormal() { return m_N; }
	Vertex* GetVertex( int a_Idx ) { return m_Vertex[a_Idx]; }
	void SetVertex( int a_Idx, Vertex* a_Vertex ) { m_Vertex[a_Idx] = a_Vertex; }
	// data members
private:
	Material* m_Material;							// 4
	int m_Type;										// 4
	// unified data for primitives
	union
	{
		// sphere
		struct
		{
			vector3 m_Centre;						// 4
			real m_SqRadius, m_Radius, m_RRadius;	// 12
			vector3 m_Ve, m_Vn, m_Vc;				// 36, total: 52
		};
		// triangle
		struct
		{
			vector3 m_N;							// 12
			Vertex* m_Vertex[3];					// 12
			real m_U, m_V;							// 8
			real nu, nv, nd;						// 12
			int k;									// 4
			real bnu, bnv;							// 8
			real cnu, cnv;							// 8, total: 64
		};
	};
};

// -----------------------------------------------------------
// Object list helper class
// -----------------------------------------------------------

class ObjectList
{
public:
	ObjectList() : m_Primitive( 0 ), m_Next( 0 ) {}
	~ObjectList() { delete m_Next; }
	void SetPrimitive( Primitive* a_Prim ) { m_Primitive = a_Prim; }
	Primitive* GetPrimitive() { return m_Primitive; }
	void SetNext( ObjectList* a_Next ) { m_Next = a_Next; }
	ObjectList* GetNext() { return m_Next; }
private:
	Primitive* m_Primitive;
	ObjectList* m_Next;
};

// -----------------------------------------------------------
// KdTree class definition
// -----------------------------------------------------------

class MManager;
class KdTreeNode
{
public:

	KdTreeNode() : m_Data( 6 ) {};
	void SetAxis( int a_Axis ) { m_Data = (m_Data & 0xfffffffc) + a_Axis; }
	int GetAxis() { return m_Data & 3; }
	void SetSplitPos( real a_Pos ) { m_Split = a_Pos; }
	real GetSplitPos() { return m_Split; }
	void SetLeft( KdTreeNode* a_Left ) { m_Data = (unsigned long)a_Left + (m_Data & 7); }
	KdTreeNode* GetLeft() { return (KdTreeNode*)(m_Data&0xfffffff8); }
	KdTreeNode* GetRight() { return ((KdTreeNode*)(m_Data&0xfffffff8)) + 1; }
	void Add( Primitive* a_Prim );
	bool IsLeaf() { return ((m_Data & 4) > 0); }
	void SetLeaf( bool a_Leaf ) { m_Data = (a_Leaf)?(m_Data|4):(m_Data&0xfffffffb); }
	ObjectList* GetList() { return (ObjectList*)(m_Data&0xfffffff8); }
	void SetList( ObjectList* a_List ) { m_Data = (unsigned long)a_List + (m_Data & 7); }
private:
	// int m_Flags;
	real m_Split;
	unsigned long m_Data;
};

struct SplitList
{
	real splitpos;
	int n1count, n2count;
	SplitList* next;
};

class KdTree
{
public:
	KdTree();
	~KdTree();
	void Build( Scene* a_Scene );
	KdTreeNode* GetRoot() { return m_Root; }
	void SetRoot( KdTreeNode* a_Root ) { m_Root = a_Root; }
	// tree generation
	void InsertSplitPos( real a_SplitPos );
	void Subdivide( KdTreeNode* a_Node, aabb& a_Box, int a_Depth, int a_Prims );
	// memory manager
	static void SetMemoryManager( MManager* a_MM ) { s_MManager = a_MM; }
private:
	KdTreeNode* m_Root;
	SplitList* m_SList, *m_SPool;
public:
	static MManager* s_MManager;
};

// -----------------------------------------------------------
// Scene class definition
// -----------------------------------------------------------

class Surface;
class Scene
{
public:
	Scene();
	~Scene();
	bool InitScene( Surface* a_MsgSurf );
	void BuildKdTree();
	int GetNrPrimitives() { return m_Primitives; }
	Primitive* GetPrimitive( int a_Idx ) { return m_Primitive[a_Idx]; }
	int GetNrLights() { return m_Lights; }
	Light* GetLight( int a_Idx ) { return m_Light[a_Idx]; }
	aabb& GetExtends() { return m_Extends; }
	KdTree* GetKdTree() { return m_KdTree; }
private:
	int m_Primitives, m_Lights;
	Primitive** m_Primitive;
	Light** m_Light;
	aabb m_Extends;
	KdTree* m_KdTree;
	// state variable to split the init in bits
	int m_State;
public:
	// scene construction methods
	void AddBox( vector3 a_Pos, vector3 a_Size );
	void AddPlane( vector3 a_P1, vector3 a_P2, vector3 a_P3, vector3 a_P4, Material* a_Mat );
	void Load3DS( char* filename, Material* a_Material, vector3& a_Pos, real a_Size );
	long EatChunk( char* buffer );
private:
	// variables for the 3ds loader
	float* m_Verts, *m_TCoords;
	int m_NrVerts, m_NrFaces;
	unsigned short* m_Faces;
};

}; // namespace Raytracer

#endif