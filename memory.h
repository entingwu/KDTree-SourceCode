
#ifndef I_MEMORY_H
#define I_MEMORY_H

namespace Raytracer {

class ObjectList;
class KdTreeNode;
class MManager
{
public:
	MManager();
	ObjectList* NewObjectList();
	void FreeObjectList( ObjectList* a_List );
	KdTreeNode* NewKdTreeNodePair();
private:
	ObjectList* m_OList;
	char* m_KdArray, *m_ObjArray;
	KdTreeNode* m_KdPtr;
	ObjectList* m_ObjPtr;
	int m_KdUsed;
};

}; // namespace Raytracer

#endif