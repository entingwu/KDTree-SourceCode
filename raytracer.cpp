#include "raytracer.h"
#include "scene.h"
#include "common.h"
#include "windows.h"
#include "winbase.h"
#include "memory.h"

namespace Raytracer {

Ray::Ray( vector3& a_Origin, vector3& a_Dir ) : 
	m_Origin( a_Origin ), 
	m_Direction( a_Dir )
{
}

Engine::Engine()
{
	m_Scene = new Scene();
	KdTree::SetMemoryManager( new MManager() );
	m_Mod = new int[64];
	m_Mod = (int*)((((unsigned long)m_Mod) + 32) & (0xffffffff - 31));
	m_Mod[0] = 0, m_Mod[1] = 1, m_Mod[2] = 2, m_Mod[3] = 0, m_Mod[4] = 1;
	m_Stack = new kdstack[64];
	m_Stack = (kdstack*)((((unsigned long)m_Stack) + 32) & (0xffffffff - 31));
}

Engine::~Engine()
{
	delete m_Scene;
}

void Engine::SetTarget( Pixel* a_Dest, int a_Width, int a_Height )
{
	// set pixel buffer address & size
	m_Dest = a_Dest;
	m_Width = a_Width;
	m_Height = a_Height;
}

int Engine::FindNearest( Ray& a_Ray, real& a_Dist, Primitive*& a_Prim )
{
	real tnear = 0, tfar = a_Dist, t;
	int retval = 0;
	vector3 p1 = m_Scene->GetExtends().GetPos();
	vector3 p2 = p1 + m_Scene->GetExtends().GetSize();
	vector3 D = a_Ray.GetDirection(), O = a_Ray.GetOrigin();
	for ( int i = 0; i < 3; i++ ) if (D.entry[i] < 0) 
	{
		if (O.entry[i] < p1.entry[i]) return 0;
	}
	else if (O.entry[i] > p2.entry[i]) return 0;
	// clip ray segment to box
	for ( int i = 0; i < 3; i++ )
	{
		real pos = O.entry[i] + tfar * D.entry[i];
		if (D.entry[i] < 0)
		{
			// clip end point
			if (pos < p1.entry[i]) tfar = tnear + (tfar - tnear) * ((O.entry[i] - p1.entry[i]) / (O.entry[i] - pos));
			// clip start point
			if (O.entry[i] > p2.entry[i]) tnear += (tfar - tnear) * ((O.entry[i] - p2.entry[i]) / (tfar * D.entry[i]));
		}
		else
		{
			// clip end point
			if (pos > p2.entry[i]) tfar = tnear + (tfar - tnear) * ((p2.entry[i] - O.entry[i]) / (pos - O.entry[i]));
			// clip start point
			if (O.entry[i] < p1.entry[i]) tnear += (tfar - tnear) * ((p1.entry[i] - O.entry[i]) / (tfar * D.entry[i]));
		}
		if (tnear > tfar) return 0;
	}
	// init stack
	int entrypoint = 0, exitpoint = 1;
	// init traversal
	KdTreeNode* farchild, *currnode;
	currnode = m_Scene->GetKdTree()->GetRoot();
	m_Stack[entrypoint].t = tnear;
	if (tnear > 0.0f) m_Stack[entrypoint].pb = O + D * tnear;
				 else m_Stack[entrypoint].pb = O;
	m_Stack[exitpoint].t = tfar;
	m_Stack[exitpoint].pb = O + D * tfar;
	m_Stack[exitpoint].node = 0;
	// traverse kd-tree
	while (currnode)
	{
		while (!currnode->IsLeaf())
		{
			real splitpos = currnode->GetSplitPos();
			int axis = currnode->GetAxis();
			if (m_Stack[entrypoint].pb.entry[axis] <= splitpos)
			{
				if (m_Stack[exitpoint].pb.entry[axis] <= splitpos)
				{
					currnode = currnode->GetLeft();
					continue;
				}
				if (m_Stack[exitpoint].pb.entry[axis] == splitpos)
				{
					currnode = currnode->GetRight();
					continue;
				}
				currnode = currnode->GetLeft();
				farchild = currnode + 1; // GetRight();
			}
			else
			{
				if (m_Stack[exitpoint].pb.entry[axis] > splitpos)
				{
					currnode = currnode->GetRight();
					continue;
				}
				farchild = currnode->GetLeft();
				currnode = farchild + 1; // GetRight();
			}
			t = (splitpos - O.entry[axis]) / D.entry[axis];
			int tmp = exitpoint++;
			if (exitpoint == entrypoint) exitpoint++;
			m_Stack[exitpoint].prev = tmp;
			m_Stack[exitpoint].t = t;
			m_Stack[exitpoint].node = farchild;
			m_Stack[exitpoint].pb.entry[axis] = splitpos;
			int nextaxis = m_Mod[axis + 1];
			int prevaxis = m_Mod[axis + 2];
			m_Stack[exitpoint].pb.entry[nextaxis] = O.entry[nextaxis] + t * D.entry[nextaxis];
			m_Stack[exitpoint].pb.entry[prevaxis] = O.entry[prevaxis] + t * D.entry[prevaxis];
		}
		ObjectList* list = currnode->GetList();
		real dist = m_Stack[exitpoint].t;
		while (list)
		{
			Primitive* pr = list->GetPrimitive();
			int result;
			m_Intersections++;
			if (result = pr->Intersect( a_Ray, dist ))
			{
				retval = result;
				a_Dist = dist;
				a_Prim = pr;
			}
			list = list->GetNext();
		}
		if (retval) return retval;
		entrypoint = exitpoint;
		currnode = m_Stack[exitpoint].node;
		exitpoint = m_Stack[entrypoint].prev;
	}
	return 0;
}


int Engine::FindOccluder( Ray& a_Ray, real& a_Dist )
{
	
	return 0;
}

Primitive* Engine::Raytrace( Ray& a_Ray, Color& a_Acc, int a_Depth, real a_RIndex, real& a_Dist)
{
	// trace primary ray
	a_Dist = 10000.0f;
	Primitive* prim = 0;
	int result;
	// find the nearest intersection
	if (!(result = FindNearest( a_Ray, a_Dist, prim ))) return 0;
	// determine color at point of intersection
	vector3 pi = a_Ray.GetOrigin() + a_Ray.GetDirection() * a_Dist;
	Color color = prim->GetColor( pi );
	vector3 N = prim->GetNormal( pi );
	// trace lights
	for ( int l = 0; l < m_Scene->GetNrLights(); l++ )
	{
		Light* light = m_Scene->GetLight( l );
		// handle point light source
		real shade = 1.0f;
		if (shade > 0)
		{
		vector3 L= light->GetPos() - pi;
		NORMALIZE(L);
			// calculate diffuse shading
			if (prim->GetMaterial()->GetDiffuse() > 0)
			{
				real dot = DOT( L, N );
				if (dot > 0)
				{
					real diff = dot * prim->GetMaterial()->GetDiffuse() * shade;
					// add diffuse component to ray color
					a_Acc += diff * color * light->GetColor();
				}
			}
		}
	}
	// calculate reflection
	real refl = prim->GetMaterial()->GetReflection();
	if ((refl > 0.0f) && (a_Depth < TRACEDEPTH))
	{
		real drefl = prim->GetMaterial()->GetDiffuseRefl();
		if ((drefl > 0) && (a_Depth < 3))
		{
		}
		else
		{
			
		}
	}
	return prim;
}

real Engine::CalcShade( Light* a_Light, vector3 a_IP, vector3& a_Dir, real a_Samples, real a_SScale )
{
	 return 0;
}
void Engine::InitRender( vector3& a_Pos, vector3& a_Target )
{
	m_CurrLine = 20;
	m_PPos = m_CurrLine * m_Width;
	/*m_Origin = vector3(  -1.0,0, 0);

	m_P1 = vector3(  0,   1.1f, 1.1 );
	m_P2 = vector3(  0,  1.1f, -1.1 );
	m_P3 = vector3( 0,   -1.1f, -1.1 );
	m_P4 = vector3( 0,   -1.1, 1.1 );*/

	m_Origin = vector3(  -2.5,0, 0.0);

	m_P1 = vector3(  0,   1.1f, 1.1 );
	m_P2 = vector3(  0,  1.1f, -1.1 );
	m_P3 = vector3( 0,   -1.1f, -1.1 );
	m_P4 = vector3( 0,   -1.1, 1.1 );



	m_DX = (m_P2 - m_P1) * (1.0f / m_Width);
	m_DY = (m_P4 - m_P1) * (1.0f / m_Height);
	// setup the tile renderer
	m_CurrCol = 0;
	m_CurrRow = 20 / TILESIZE;
	m_XTiles = m_Width / TILESIZE;
	m_YTiles = (m_Height - 40) / TILESIZE;
	// reset counters
	m_Intersections = 0;
	m_RaysCast = 0;
}
Primitive* Engine::RenderRay( vector3 a_ScreenPos, Color& a_Acc )
{
	aabb e = m_Scene->GetExtends();
	vector3 dir = a_ScreenPos - m_Origin;
	NORMALIZE( dir );
	Color acc( 0, 0, 0 );
	Ray r( m_Origin, dir );
	m_RaysCast++;
	real dist;
	// trace ray
	return Raytrace( r, a_Acc, 1, 1.0f, dist);
}

bool Engine::RenderTiles()
{
	// render scene in a tile based fashion
	aabb e = m_Scene->GetExtends();
	// initialize timer
	int msecs = GetTickCount();
	// render remaining tiles
	int tx = m_CurrCol, ty = m_CurrRow;
	int tdest = tx * TILESIZE + (ty * TILESIZE) * m_Width;
	vector3 tdir = m_P1 + (real)(tx * TILESIZE) * m_DX + (real)(ty * TILESIZE) * m_DY;
	while (1)
	{
		int dest = tdest;
		vector3 ldir = tdir;
		for ( int y = 0; y < TILESIZE; y++ )
		{
			vector3 pdir = ldir;
			for ( int x = 0; x < TILESIZE; x++ )
			{
				Color acc( 0, 0, 0 );
				Primitive* prim = RenderRay( pdir, acc );
				int red, green, blue;
				red = (int)(acc.r * 256);
				green = (int)(acc.g * 256);
				blue = (int)(acc.b * 256);
				if (red > 255) red = 255;
				if (green > 255) green = 255;
				if (blue > 255) blue = 255;
				m_Dest[dest++] = (red << 16) + (green << 8) + blue;
				pdir += m_DX;
			}
			ldir += m_DY;
			dest += (m_Width - TILESIZE);
		}
		tdest += TILESIZE;
		tdir += m_DX * TILESIZE;
		if (++tx == m_XTiles)
		{
			tx = 0;
			ty++;
			tdest = ty * TILESIZE * m_Width;
			tdir = m_P1 + (real)(ty * TILESIZE) * m_DY;
		}
		if (ty < m_YTiles)
		{
			if ((GetTickCount() - msecs) > 200) 
			{
				m_CurrCol = tx;
				m_CurrRow = ty;
				return false;
			}
		}
		else break;
	}
	return true;
}

}; // namespace Raytracer