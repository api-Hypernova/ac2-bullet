#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

//#include "LinearMath/btQuickprof.h"
//#include "LinearMath/btIDebugDraw.h"
#include "cube.h"
#include "world.h"
#include "octa.h"
#include "light.h"
#include "texture.h"
#include "bih.h"
#include "model.h"
#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"





class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class btTriangleIndexVertexArray;
//btDiscreteDynamicsWorld* m_dynamicsWorld;




extern int PHYSDebugDraw;

extern btRigidBody *setcbfrommodel(vec o, const char *mdl, vec dir, int weight);//attatch collision box to model

extern void buildLevelTriCol();// build level collision mesh

extern void PHYSInit();
extern void PHYSKill();
extern void PHYSStep();
extern void PHYSrebuildLevel();// rebuild level collision mesh
