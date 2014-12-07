

#ifndef I_RAYTRACER_H
#define I_RAYTRACER_H

#include "common.h"
#include "twister.h"

namespace Raytracer {


class Ray
{
public:
	Ray() : m_Origin( vector3( 0, 0, 0 ) ), m_Direction( vector3( 0, 0, 0 ) ) {};
	Ray( vector3& a_Origin, vector3& a_Dir );
	void SetOrigin( vector3& a_Origin ) { m_Origin = a_Origin; }
	void SetDirection( vector3& a_Direction ) { m_Direction = a_Direction; }
	vector3& GetOrigin() { return m_Origin; }
	vector3& GetDirection() { return m_Direction; }
private:
	vector3 m_Origin;
	vector3 m_Direction;
};

// -----------------------------------------------------------
// Engine class definition
// Raytracer core
// -----------------------------------------------------------
class KdTreeNode;
struct kdstack
{
	KdTreeNode* node;
	real t;
	vector3 pb;
	int prev, dummy1, dummy2;
};

class Scene;
class Primitive;
class Light;
class Engine
{
public:
	Engine();
	~Engine();
	void SetTarget( Pixel* a_Dest, int a_Width, int a_Height );
	Scene* GetScene() { return m_Scene; }
	int FindNearest( Ray& a_Ray, real& a_Dist, Primitive*& a_Prim );
	int FindOccluder( Ray& a_Ray, real& a_Dist );
	Primitive* Raytrace( Ray& a_Ray, Color& a_Acc, int a_Depth, real a_RIndex, real& a_Dist);
	real CalcShade( Light* a_Light, vector3 a_IP, vector3& a_Dir, real a_Samples, real a_SScale );
	void InitRender( vector3& a_Pos, vector3& a_Target );
	Primitive* RenderRay( vector3 a_ScreenPos, Color& a_Acc );
	bool RenderTiles();
protected:
	// renderer data
	Scene* m_Scene;
	Pixel* m_Dest;
	int m_Width, m_Height, m_CurrLine, m_PPos;
	//Twister m_Twister;
	vector3 m_Origin, m_P1, m_P2, m_P3, m_P4, m_DX, m_DY;
	int* m_Mod;
	kdstack* m_Stack;
	// tile renderer data
	int m_CurrCol, m_CurrRow;
	int m_XTiles, m_YTiles;
public:
	// counters
	int m_RaysCast, m_Intersections;
};

}; // namespace Raytracer

#endif