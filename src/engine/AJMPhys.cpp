#include "cube.h"

#include "engine.h"
#include "game.h"

//#include "LinearMath/btAabbUtil2.h"
//#include "LinearMath/btMatrix3x3.h"
/////includes for bullet are in cube.h
/////disabled new and delete to resolve conflict with cpp new and delete included with bullet/////
//////////////////////////bullet//////////////////////////////////////////////////////
//       Lets grab our verts for the world and build ourselves a collision mesh  /////
//       search fro "//angelo phys bullet" in the rest of the code to see changes ////
//////////////////////////////////////////////////////////////////////////////////////

btTriangleIndexVertexArray* m_indexVertexArrays;

//btStridingMeshInterface* m_indexVertexArrays;
btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
btBroadphaseInterface*	m_broadphase;
btCollisionDispatcher*	m_dispatcher;
btConstraintSolver*	m_solver;
btDefaultCollisionConfiguration* m_collisionConfiguration;
btDiscreteDynamicsWorld* m_dynamicsWorld;

btBvhTriangleMeshShape* trimeshShape;
btCollisionShape* groundShape;

//btRigidBody* body;
btVector3 worldMin(-1000,-1000,-1000);
btVector3 worldMax(1000,1000,1000);

VAR(PHYSDebugDraw, 0, 1, 1);

#define SCALING 0.058825
int matnumber = 0;

void buildLevelTriCol()
{
    int VertexCount = 0;
    if (m_dynamicsWorld){//////dont build if phyics arent running
      
    renderprogress(0, "computing levelCollision...");

    //from the writeobj function
    //with a couple of additions and subtreactions from me
    extern vector<vtxarray *> valist;
    vector<vec> verts, texcoords;
    hashtable<vec, int> shareverts(1<<16), sharetc(1<<16);
    hashtable<int, vector<ivec2> > mtls(1<<8);
    vector<int> usedmtl;
    vec bbmin(1e16f, 1e16f, 1e16f), bbmax(-1e16f, -1e16f, -1e16f);//might need these for the aabbMax/aabbMin below ?
    loopv(valist)
    {
        vtxarray &va = *valist[i];
        if(!va.edata || !va.vdata) continue;
        ushort *edata = va.edata + va.eoffset;
        vertex *vdata = va.vdata;
        ushort *idx = edata;
        loopj(va.texs)
        {
            elementset &es = va.texelems[j];
            if(usedmtl.find(es.texture) < 0) usedmtl.add(es.texture);
            vector<ivec2> &keys = mtls[es.texture];
            loopk(es.length)
            {
               const vertex &v = vdata[idx[k]];
                const vec &pos = v.pos;
                const vec &tc = v.tc;
                ivec2 &key = keys.add();
                key.x = shareverts.access(pos, verts.length());
                if(key.x == verts.length())
                {
                    verts.add(pos);
                    bbmin.min(pos);
                    bbmax.max(pos);
		    VertexCount += 1;//get vert count 
                }
                key.y = sharetc.access(tc, texcoords.length());
                if(key.y == texcoords.length()) texcoords.add(tc);
            }
            idx += es.length;
        }
    } 
    int vertStride = sizeof(btVector3);
    int indexStride = 3*sizeof(int);    
    //conoutf("Level VertexCount %d\r\n", VertexCount);
    btVector3* Vertices = new btVector3[VertexCount];
    //fill in our Vertices locations
    vec center(-(bbmax.x + bbmin.x)/2, -(bbmax.y + bbmin.y)/2, -bbmin.z);
    //conoutf("adding vert locations");
    renderprogress(0, "adding vert locations %d verts", VertexCount);
    
    loopv(verts)
    {
        vec v = verts[i];
        v.add(center);
	if(v.y != floor(v.y)) Vertices[i][0] =-v.y; else Vertices[i][0] =int(-v.y);
	if(v.z != floor(v.z)) Vertices[i][1] =v.z; else Vertices[i][1] =int(v.z);
	if(v.x != floor(v.x)) Vertices[i][2] =v.x; else Vertices[i][2] =int(v.x);
	//conoutf("Level verts %.3f/%.3f/%.3f\r\n", Vertices[i][0],Vertices[i][1],Vertices[i][2]);
    }

    //this first count seems silly but for now its how i get my size for the index container
    //it runs fast enough that i am not going to worry with it for now
    //works great until you add another material to the mix:( <<<<<<<< i think this is fixed :)
    //have to run 2 searches, in someone can do this in a more simple maner, please share.

    usedmtl.sort();
    int IndexCount = 0;//counter to keep track of indices while they are being filled
    int TriCount = 0;//amount of total triangles
    int* Indices = 0;//final indice array
    
    //count tris from each material
    //conoutf("counting tris");
    loopv(usedmtl)
    { 
        vector<ivec2> &keys = mtls[usedmtl[i]];      
        for(int tc = 0; tc < keys.length(); tc += 3)
        {
	 TriCount += 1;//amount of total triangles to send to the indices array
	}
    }
    //conoutf("Level TriCount %d\r\n", TriCount);
    
    Indices = new int[TriCount*3];
    //now we fill our Indeces
    //conoutf("adding tri indices");
    renderprogress(0, "adding tri indices %d triangles", TriCount);
    loopv(usedmtl)
    {
        vector<ivec2> &keys = mtls[usedmtl[i]];
        for(int ic = 0; ic < keys.length(); ic += 1)
        {
	  //Indices[i] = keys[ic].x;//+1;//indeces should start at 0?//yes, yes they should
	  Indices[IndexCount] = keys[ic].x;
	  //conoutf("Level Indice %d\r\n", Indices[IndexCount]);
	  IndexCount += 1;//simple yet elegent :)
        }
    }
    //conoutf("data collected, adding it to the array");
    renderprogress(0, "data collected, adding it to the array");// %d verts, %d triangles", VertexCount,TriCount);
	m_indexVertexArrays = new btTriangleIndexVertexArray(TriCount,
		Indices,
		indexStride,
		VertexCount,(btScalar*) &Vertices[0].x(),vertStride);
	
	bool useQuantizedAabbCompression = true;

	//btVector3 aabbMin(1e16f, 1e16f, 1e16f), aabbMax(-1e16f, -1e16f, -1e16f);//nonono dont use there will beno collision
	btVector3 aabbMin(-1000,-1000,-1000), aabbMax(1000,1000,1000);
	trimeshShape  = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression,aabbMin,aabbMax);
   
	btCollisionShape* groundShape = trimeshShape;
	groundShape->setLocalScaling(btVector3(0.058825,0.058825,0.058825));//scaling from bullet obj to cube TODO:make this easier

	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(25.),btScalar(5.),btScalar(25.)));//used to make sure bullet was running
	//m_collisionShapes.push_back(groundShape);//it dont like this when we try to clean up :(

	btTransform groundTransform;
	groundTransform.setIdentity();

	groundTransform.setOrigin(btVector3(center.y*-0.058825,center.z*-0.058825,center.x*-0.058825));//put col mesh in proper location
	//conoutf("center %.3f/%.3f/%.3f\r\n", center.y*-0.058825,center.z*-0.058825,center.x*-0.058825);
	groundTransform.setRotation(btQuaternion(0,1,0,1));//done in radians not degrees?very wierd 1 = 90.........
	/*
	btCollisionObject* fixedGround = new btCollisionObject();
	fixedGround->setCollisionShape(groundShape);
	fixedGround->setWorldTransform(groundTransform);
	m_dynamicsWorld->addCollisionObject(fixedGround);
	*/
	
	///may need to add as rigid body?
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
        btScalar mass(0.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		if(body)conoutf("there is a ground body");

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}
	
    renderprogress(0, "Level Collision mesh primed and ready");
    }
    else { conoutf("NONONO!! No Secondary Physics Running!");}

}
COMMAND(buildLevelTriCol, "s");//for testing purposes

//temp borrow from intencity engine
btIDebugDraw* m_debugDrawer;
// Sauer coordinates are in 'cubes', not metres as in Bullet
#define SAUER_FACTOR 17.0
// Reverse y and z axes
#define FROM_SAUER_VEC(sauervec) ( btVector3(sauervec.x/SAUER_FACTOR, sauervec.z/SAUER_FACTOR, sauervec.y/SAUER_FACTOR) )
#define TO_SAUER_VEC(sauervec, btvec) { sauervec.x = btvec.x()*SAUER_FACTOR; sauervec.y = btvec.z()*SAUER_FACTOR; sauervec.z = btvec.y()*SAUER_FACTOR; }

class SauerDebugDrawer : public btIDebugDraw
{
    int m_debugMode;
public:
    SauerDebugDrawer() : m_debugMode(0) { };
    virtual void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &fromColor, const btVector3 &toColor)
    {
	vec sauerFrom, sauerTo;
	TO_SAUER_VEC( sauerFrom, from );
	TO_SAUER_VEC( sauerTo, to );

	#define VEC_TO_COLOR(it) \
	    ((int((it.x()*0.5+0.5)*255)<<16) + (int((it.y()*0.5+0.5)*255)<<8) + int((it.z()*0.5+0.5)*255))
	particle_flare(sauerFrom, sauerTo, 0, PART_STREAK, VEC_TO_COLOR(fromColor));//,1);

    }

    virtual void drawLine(const btVector3 &from, const btVector3 &to, const btVector3 &color)
    {
	drawLine(from, to, color, color);

    }
        virtual void draw3dText (const btVector3 &location, const char *textString)
        {
            vec sauerLocation;
            TO_SAUER_VEC( sauerLocation, location );
            particle_text(sauerLocation, textString, PART_TEXT, 0);
        }
        virtual void drawContactPoint (const btVector3 &PointOnB, const btVector3 &normalOnB, btScalar distance, int lifeTime, const btVector3 &color) { }
        virtual void reportErrorWarning (const char *warningString) { printf("Bullet warning: %s\r\n", warningString); }
        virtual void setDebugMode(int debugMode) { m_debugMode = debugMode; }
        virtual int getDebugMode() const { return m_debugMode; }
};
/////////^^^^^^^^^^^^temp borrow from intencity engine^^^^^^^^^^^^^^^^/////////////////////////


///The MyOverlapCallback is used to show how to collect object that overlap with a given bounding box defined by aabbMin and aabbMax. 
///See m_physicsSetup.m_dynamicsWorld->getBroadphase()->aabbTest.
struct	MyOverlapCallback : public btBroadphaseAabbCallback
{
	btVector3 m_queryAabbMin;
	btVector3 m_queryAabbMax;
	
	int m_numOverlap;
	MyOverlapCallback(const btVector3& aabbMin, const btVector3& aabbMax ) : m_queryAabbMin(aabbMin),m_queryAabbMax(aabbMax),m_numOverlap(0)	{}
	virtual bool	process(const btBroadphaseProxy* proxy)
	{
		btVector3 proxyAabbMin,proxyAabbMax;
		btCollisionObject* colObj0 = (btCollisionObject*)proxy->m_clientObject;
		colObj0->getCollisionShape()->getAabb(colObj0->getWorldTransform(),proxyAabbMin,proxyAabbMax);
		if (TestAabbAgainstAabb2(proxyAabbMin,proxyAabbMax,m_queryAabbMin,m_queryAabbMax))
		{
			m_numOverlap++;
		}
		return true;
	}
};

btRigidBody* bodypl = 0;
void addplayertobullet(){
	//player bounding box
	btCollisionShape* colShape = new btCapsuleShape(player->radius*SCALING, (player->eyeheight - (player->aboveeye * 2))*SCALING);
	colShape->setMargin(btScalar(0.04f));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	btScalar mass(8);
	btVector3 localInertia(0, 0, 1);

	colShape->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0f, 0.0f, 0.0f)));

	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	rbInfo.m_angularSleepingThreshold = btScalar(0.0f);
	rbInfo.m_linearSleepingThreshold = btScalar(0.0f);

	btRigidBody* body = new btRigidBody(rbInfo);

	btTransform startTransform;
	startTransform.setIdentity();

	startTransform.setOrigin(btVector3(player->o.x*0.058825, player->o.z*0.058825, player->o.y*0.058825));

	body->setCenterOfMassTransform(startTransform);
	body->setAngularFactor(btVector3(0, 0, 0));
	body->setLinearFactor(btVector3(1, 1, 1));
	body->setDamping(0.9, 0.3);
	body->setRestitution(0.0f);
	body->forceActivationState(DISABLE_DEACTIVATION); //stops object from sleeping

	body->setWorldTransform(startTransform);

	body->setUserIndex(-1);
	m_dynamicsWorld->addRigidBody(body);
	bodypl = body;
}

void addPhysicEntToBullet(physent *pl)
{
    //player bounding box
    btCollisionShape* colShape = new btCapsuleShape(pl->radius*SCALING, (pl->eyeheight - (pl->aboveeye * 2))*SCALING);
    colShape->setMargin(btScalar(0.04f)); //set this margin larger if object jitters

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    btScalar mass(pl->weight); //set the mass to pl wieght
    btVector3 localInertia(0, 0, 1); //dont screw with this :P

    colShape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* myMotionState = \
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0f, 0.0f, 0.0f)));

    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    rbInfo.m_angularSleepingThreshold = btScalar(0.0f);
    rbInfo.m_linearSleepingThreshold = btScalar(0.0f);

    btRigidBody* body = new btRigidBody(rbInfo);

    btTransform startTransform;
    startTransform.setIdentity();

    startTransform.setOrigin(btVector3(player->o.x*0.058825, player->o.z*0.058825, player->o.y*0.058825));

    body->setCenterOfMassTransform(startTransform);
    body->setAngularFactor(btVector3(0, 0, 0)); //dont let the object spin (cuz we will just move the camera
    body->setLinearFactor(btVector3(1, 1, 1)); //but we want to be able to move in all directions
    body->setDamping(0.9, 0.0); //Damping is how much momentum we have from last frame
              // so we keep 10% of the momentum from last frame
    body->setRestitution(0.0f); //we dont what the body to bounce, cuz its a person not a ball.
    body->forceActivationState(DISABLE_DEACTIVATION); //stops object from sleeping

    body->setWorldTransform(startTransform);
        body->setGravity(btVector3(0.f, 0, 0)); //we are gonna manually handle gravity
    m_dynamicsWorld->addRigidBody(body);
        pl->body = body;
}

inline btVector3 convert2bt(vec vt){ vec v(vt); v.mul(SCALING); return btVector3(v.x, v.z, v.y); }
inline vec convertfrombt(btVector3 &v){ return vec(v.getX(), v.getZ(), v.getY()).mul(SAUER_FACTOR); }


bool rayTest(vec start, vec end, vec &normal)
{
    btCollisionWorld::ClosestRayResultCallback RayCallback(convert2bt(start), convert2bt(end));
    m_dynamicsWorld->rayTest(convert2bt(start), convert2bt(end), RayCallback);
    if (RayCallback.hasHit()) //we hit something YAY
       {
        end = convertfrombt(RayCallback.m_hitPointWorld); //lets get where the vector hit
        normal = convertfrombt(RayCallback.m_hitNormalWorld); //lets get the normal of that plane
        return true;
    }
    return false;
}


bool canjump(physent *pl)
{
    if (pl->type != ENT_PLAYER && (pl->state != CS_ALIVE)) return false;
                  //is the object alive and is the object a player
    vec v(pl->feetpos()); //lets get the players feet so we can see if hes on the ground
    vec n(0); //temp vec
    if (rayTest(vec(v), vec(v).addz(-1.0), n)) //so lets look at our feet and then 1 unit beyond that
        {
            n.normalize();
           //so we want to get a directional vector (look up normalization if you dont know what this is)
            pl->physstate = n.absdot(vec(0, 0, 1.f)) == 1.0f ? PHYS_FLOOR : PHYS_SLOPE;
                 //ok so what is the angel of what what we are standing on
            pl->floor = n; //ok tell pl the normal of the floor (so we can have fun with it later;
            return true;
        }
//so now we know if the object is directly below us but what if we are on some weird slope (or in a hole)
    loopi(3) //loop once for the x
        loopk(3)//loop once for the y
        if (rayTest(vec(v), vec(v).add(vec(i - 1, k - 1, -1.5)), n))
         //do 1.5 times that length cuz we are looking at an angel
        //so we are gonna look in every -z direction to see if maybe there is an object that we are on.
        {
         pl->physstate = PHYS_SLOPE; //if so we are on a slope
         pl->floor = n; //set the floor
         return true;
        }
    pl->physstate = PHYS_FALL; //else we must be in the air 0.0 OOOO NO
    return false;
}

VAR(floatspeed, 1, 100, 10000);

void btmoveplayer(physent *pl, int currenttime)
{
    int material = lookupmaterial(vec(pl->o.x, pl->o.y, pl->o.z + (3 * pl->aboveeye - pl->eyeheight) / 4));
    bool water = isliquid(material&MATF_VOLUME); //are we in water
    bool floating = pl->type == ENT_PLAYER&&(pl->state == CS_EDITING||pl->state == CS_SPECTATOR);
          //are we in float mode
    float secs = curtime / 1000.f; //get the time so we can do the multiplying correctly
    bool inair = !canjump(pl); //can we jump 0.0
    float jumpheight = 200.f; //we may need to play with this
    pl->jumping = pl->jumping && !inair;
    //floating = true;
        int maxspeed = 500; //make sure that the object isnt moving too fast for bullet.
    vec d (0, 0, 0);
    if (!floating && water) d.mul(0.5f); //add fake water friction
    pl->vel = vec(0);
        //so we dont want to carry and momentum over from last frame
    vecfromyawpitch(pl->yaw, floating||water||pl->type == ENT_CAMERA ? \
                   pl->pitch : 0, pl->move, pl->strafe, d);
        //get the direction of the camera and store it in d
    d.mul(pl->maxspeed);
    if (pl->state == CS_EDITING){

        pl->body->forceActivationState(DISABLE_SIMULATION);//we dont wanna clip on anything
        pl->body->setFriction(0.f);
        pl->body->setDamping(0.0, 0.0);
                //pl->body->setLinearVelocity(convert2bt(d*secs*floatspeed*0.01));
                pl->body->setLinearVelocity(convert2bt(vec(d).mul(secs*floatspeed*0.01)));
        return;
    }
    else if (pl->state == CS_SPECTATOR){
        pl->body->forceActivationState(DISABLE_DEACTIVATION);//we want to clip on everything
        pl->body->setGravity(btVector3(0.f, 0, 0));
                pl->body->setDamping(0.0, 0.0);
                pl->body->setFriction(0.0f);
        pl->body->setLinearVelocity(convert2bt(vec(d).mul(secs)));
    }
    else if (game::allowmove(pl)) // is it paused or intermission
        {
                pl->body->forceActivationState(DISABLE_DEACTIVATION); //stop the player from deactivation
                pl->body->setDamping(0.9, 0.0);
                pl->body->setFriction(pl->physstate == PHYS_FALL ? 0.1 //if we are falling add air friction
                 : pl->physstate == PHYS_SLOPE&&(!pl->move && !pl->strafe) ?0.4f \
                    //are we on a slope and we are not moving
                 : 0.2f); //if we are on a slope moving, or on the ground then we need to set normal friction
        if (pl->physstate == PHYS_FALL)
                {
                        pl->body->setDamping(0.3, 0.0); //we want less damping when we are in the air
            d.mul(0.2f); //we want to stop the player from moving as fair

            if (pl->timeinair > 2000)pl->timeinair = 2000;
                          //cap your make speed from gravity so we dont fall through the map
            if (pl->timeinair > 100)
            {
                //pl->body->setLinearVelocity(convert2bt(0.f, -pow(30.48f, pl->timeinair*0.001), 0)); //handle gravity
                pl->body->setGravity(btVector3(0.f, -20, 0));
            }
            pl->timeinair += curtime;//add to the timeinair
        }
        else{ pl->timeinair = 0; }
        if (pl->jumping){
            pl->jumping = false;
            d.addz(jumpheight);
            game::physicstrigger(pl, true, 1, 0);
        }
                d.add(pl->resilientvel*secs); //apply the left over velocity
                pl->resilientvel.mul(0.2/secs); //decay over time
                loopi(3)clamp(-maxspeed, d[i], (float)maxspeed); //make sure that no direction is moving too fast
                //pl->body->setLinearVelocity(convert2bt(d.mul(secs)), btVector3(0, 0, 0)); //set the velocity
                pl->body->setLinearVelocity(convert2bt(vec(d).mul(secs)));
                //NOTE: never call applyLinearImpulse because this will be ignored, rather set resilientvel
                // in the case of a jumppad pl->resilientvel = vec(attr1, attr2, attr3);
    }

}

//void btmoveplayer(physent *pl, int curenttime){
//    int material = lookupmaterial(vec(pl->o.x, pl->o.y, pl->o.z + (3 * pl->aboveeye - pl->eyeheight) / 4));
//    bool water = isliquid(material&MATF_VOLUME);
//    bool floating = pl->type == ENT_PLAYER && (pl->state == CS_EDITING || pl->state == CS_SPECTATOR);
//    //float secs = curtime / 1000.f;
//    bool inair = !canjump(pl);
//    pl->jumping = pl->jumping && !inair;
//    //floating = true;
//    vec d(0, 0, 0);
//    //if (!floating && water) d.mul(0.5f);
//    pl->vel = vec(0);
//    vecfromyawpitch(pl->yaw, floating || water || pl->type == ENT_CAMERA ? pl->pitch : 0, pl->move, pl->strafe, d);
//    d.mul(40);
//    if (pl->state == CS_EDITING){
//        pl->o.add(d.mul(0.025*floatspeed*0.01));
//        bodypl->forceActivationState(DISABLE_SIMULATION);
//        bodypl->setFriction(0.f);
//        bodypl->setDamping(0.9, 0.0);
//    }
//    else if (pl->state == CS_SPECTATOR){
//        bodypl->forceActivationState(DISABLE_DEACTIVATION);
//        bodypl->setGravity(btVector3(0.f, 0, 0));
//        bodypl->setDamping(0.9, 0.0);
//        bodypl->setFriction(0.f);
//        bodypl->applyImpulse(btVector3(d.x*SCALING, d.z*SCALING, d.y*SCALING), btVector3(0, 0, 0));
//    }
//    else if (game::allowmove(pl)){
//        bodypl->forceActivationState(DISABLE_DEACTIVATION);
//        bodypl->setDamping(0.9, 0.0);
//        //bodypl->setGravity(btVector3(0.f, -20, 0));
//        bodypl->setFriction(pl->physstate == PHYS_FALL ? 0.1 : pl->physstate == PHYS_SLOPE ? 0.0f : 0.3);
//        if (pl->physstate == PHYS_FALL){
//            bodypl->setDamping(0.3, 0.0);
//            d.mul(0.2f);
//        }
//        if (pl->physstate == PHYS_SLOPE){
//            bodypl->setFriction((pl->move != 0 || pl->strafe != 0 ? 0.2 : 0.9));
//            d.mul(vec(1.f, 1.f, pl->floor.z * 10.f));
//        }
//        if (pl->jumping){
//            pl->jumping = false;

//            d.addz(530 * 2);
//            bodypl->setDamping(0.1, 0.0);
//            game::physicstrigger(pl, true, 1, 0);
//        }
//        //conoutf("%d", pl->physstate);
//        bodypl->applyImpulse(btVector3(d.x*SCALING, d.z*SCALING, d.y*SCALING), btVector3(0, 0, 0));
//    }

//}

void PHYSInit(){
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);


	//btVector3 worldMin(-1000,-1000,-1000);
	//btVector3 worldMax(1000,1000,1000);
	//m_broadphase = new btAxisSweep3(worldMin,worldMax);
	m_broadphase = new btDbvtBroadphase();

	m_solver = new btSequentialImpulseConstraintSolver();

	
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	
	
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));//make this level independent
	
	////////temp borrow
        m_debugDrawer = new SauerDebugDrawer();
        m_debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
        m_dynamicsWorld->setDebugDrawer(m_debugDrawer);
	////////temp borrow^^^^^
	
	m_indexVertexArrays = NULL;
	body = NULL;
	conoutf("Secondary Physics engine Enabled");
	addplayertobullet();
}
COMMAND(PHYSInit, "s");//for testing purposes

void PHYSKill()
{
	if (m_dynamicsWorld)
	{  
	//cleanup in the reverse order of creation/initialization
	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}
	//delete collision shapes
	if(m_collisionShapes.size() !=0){
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		if(shape){
		delete shape;
		}
	}
	}
	//delete dynamics world
	delete m_dynamicsWorld;
	if (m_indexVertexArrays)
		delete m_indexVertexArrays;
	//delete solver
	delete m_solver;
	//delete broadphase
	delete m_broadphase;
	//delete dispatcher
	delete m_dispatcher;
	delete m_collisionConfiguration;
        //delete groundShape;
        //delete trimeshShape;
        delete m_debugDrawer;
	//reset our containers
	m_dynamicsWorld = 0;
	m_solver = 0;
	m_broadphase = 0;
	m_dispatcher = 0;
	m_collisionConfiguration = 0;
	groundShape = 0;
	trimeshShape = 0;
        conoutf("Secondary Physics engine Disabled");
	}
}
COMMAND(PHYSKill, "s");//for testing purposes

//http://www.irrlicht3d.org/wiki/index.php?n=Main.GettingStartedWithBullet
// Converts a quaternion to an euler angle
void QuaternionToEuler(const btQuaternion &TQuat, vec &TEuler) {
	float W = TQuat.getW();
	float X = TQuat.getX();
	float Z = -TQuat.getY();
	float Y = TQuat.getZ();
	float WSquared = W * W;
	float XSquared = X * X;
	float YSquared = Y * Y;
	float ZSquared = Z * Z;

	TEuler.x=(atan2f(2.0f * (Y * Z + X * W), -XSquared - YSquared + ZSquared + WSquared));
	TEuler.y=(asinf(-2.0f * (X * Z - Y * W)));
	TEuler.z=(atan2f(2.0f * (X * Y + Z * W), XSquared - YSquared - ZSquared + WSquared));
	TEuler.mul( 180.0f / PI);//90,180,270
}

void PHYSStep(){
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(lastmillis / 100.f);
		//btTransform trans;
		//if(!curtime) return;

		btmoveplayer(player, 0);

		//print positions of all objects
		//for (int j=m_dynamicsWorld->getNumCollisionObjects()-1; j < 0 ;j--)
		//{
		//	btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
		//	btRigidBody* body = btRigidBody::upcast(obj);
		//	if (body && body->getMotionState())
		//	{
		//		vec tmpvec;
		//		vec tmprot;
		//		btTransform trans;
		//		body->getMotionState()->getWorldTransform(trans);

		//		btVector3 EulerRotation;
		//		QuaternionToEuler(body->getOrientation(), EulerRotation);
		//		//conoutf("world pos = %f,%f,%f\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
		//		//tmpvec = vec(float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()));
		//		//tmprot = vec(float(trans.getRotation().getX()),float(trans.getRotation().getY()),float(trans.getRotation().getZ()));
		//		tmpvec = vec(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ());
		//		//tmprot = vec(trans.getRotation().getX(),trans.getRotation().getY(),trans.getRotation().getZ());
		//		tmprot = vec(EulerRotation.getX(), EulerRotation.getY(), EulerRotation.getZ());
		//		//trans.
		//		game::updatebulletmovables(tmpvec,tmprot,j);
		//	}
		//	
		//}
		
		game::updatebulletmovables(vec(1), vec(1), 1);
		//m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		//m_physicsSetup.m_dynamicsWorld->debugDrawWorld();

		btVector3 aabbMin(1,1,1);
		btVector3 aabbMax(2,2,2);

		MyOverlapCallback aabbOverlap(aabbMin,aabbMax);
		m_dynamicsWorld->getBroadphase()->aabbTest(aabbMin,aabbMax,aabbOverlap);
		
		//if (aabbOverlap.m_numOverlap)
			//conoutf("#aabb overlap = %d\n", aabbOverlap.m_numOverlap);
		if(PHYSDebugDraw){
                m_debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
                m_dynamicsWorld->debugDrawWorld();
		}
		//conoutf("dworld stepped through");
		if (!bodypl) return;
		if (player->state == CS_EDITING)return;
		btTransform trans;
		vec tmpvec;
		bodypl->getMotionState()->getWorldTransform(trans);
		tmpvec = vec(trans.getOrigin().getX(), trans.getOrigin().getZ(), trans.getOrigin().getY());
		player->o = tmpvec.mul(17).addz((player->maxheight - player->aboveeye * 2)*0.5);
	}  
}
COMMAND(PHYSStep, "s");//for testing purposes
void PHYSrebuildLevel(){
	if (m_dynamicsWorld)
	{
	 PHYSKill();
	 PHYSInit();
	 buildLevelTriCol();
	 
	 conoutf("Level collision rebuilt");
	}
}
btQuaternion rotate(float yaw, float pitch, float roll) {
	// Assuming the angles are in radians.
	float c1 = cosf(yaw);
	float s1 = sinf(yaw);
	float c2 = cosf(pitch);
	float s2 = sinf(pitch);
	float c3 = cosf(roll);
	float s3 = sinf(roll);
	float w = sqrt(1.0 + c1 * c2 + c1*c3 - s1 * s2 * s3 + c2*c3) / 2.0;
	float w4 = (4.0 * w);
	return btQuaternion(
		(c2 * s3 + c1 * s3 + s1 * s2 * c3) / w4,   /*x*/
		(s1 * c2 + s1 * c3 + c1 * s2 * s3) / w4,  /*y*/
		(-s1 * s3 + c1 * s2 * c3 + s2) / w4,     /*z*/
		w										/*w*/
		);
}

COMMAND(PHYSrebuildLevel, "s");//for testing purposes
btVector3 impulsevec = btVector3(0, 0, 0);
ICOMMAND(setangleimpulse, "fff", (float *x, float *y, float *z), { impulsevec = btVector3(*x, *y, *z); });
btRigidBody *setcbfrommodel(vec o, const char *mdl, vec dir, int weight)//set collision box
{
	if (m_dynamicsWorld)
	{

		model *m = loadmodel(mdl);
		if (!m) return NULL;
		vec center, radius;
		m->collisionbox(center, radius);
		//center.z -= radius.z;
		o = center.add(o);
		//if(m->collide != COLLIDE_ELLIPSE) d->collidetype = COLLIDE_OBB;
		
		//d->radius    = max(d->xradius, d->yradius);//sqrtf(d->xradius*d->xradius + d->yradius*d->yradius);
		//d->eyeheight = (center.z-radius.z) + radius.z*2*m->eyeheight;
		//d->aboveeye  = radius.z*2*(1.0f-m->eyeheight);    
		///scaling of the objects (0.1 = 20 centimeter boxes )


		// 1. Create collision shape
		// Re-using the same collision shape is better for memory usage and performance
		//btCollisionShape* colShape = new btBoxShape(btVector3(SCALING * 1.2f, SCALING * 1.2f, SCALING * 1.2f));
		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING * (radius.y), SCALING * (radius.z), SCALING * (radius.x)));
		// 2. Set some properties of collision shape
		colShape->setMargin(btScalar(0.04f));
		bool staticobj = false;
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		if (weight < 0){ weight = 0; staticobj = true;}
		btScalar mass(weight);
		btVector3 localInertia(0, 0, 1);

		colShape->calculateLocalInertia(mass, localInertia);

		// 3. Create motions state for rigid body
		btDefaultMotionState* myMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0.0f, 0.0f, 0.0f)));

		// 4. Create rigid body
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		rbInfo.m_angularSleepingThreshold = btScalar(0.0f);
		rbInfo.m_linearSleepingThreshold = btScalar(0.0f);

		btRigidBody* body = new btRigidBody(rbInfo);

		// 5. Set properties of the rigid body
		//d.feetpos().
		btTransform startTransform;
		startTransform.setIdentity();

		//startTransform.setOrigin(btVector3(30,50,30));//(float(d->o.x),float(d->o.y),float(d->o.z)));*-0.058825
		//startTransform.setOrigin(btVector3(d->o.x*0.058825,d->o.z*0.058825,d->o.y*0.058825)); //have to scale location
		startTransform.setRotation(btQuaternion(rotate(dir.x*RAD, dir.y*RAD, dir.z*RAD)));
		startTransform.setOrigin(btVector3(center.x*0.058825, center.z*0.058825, center.y*0.058825 )); //have to scale location
		//conoutf("world pos = %f,%f,%f\n",float(d->feetpos().y),float(d->feetpos().z),float(d->feetpos().x));
		
		body->setCenterOfMassTransform(startTransform);
		body->setAngularFactor(btVector3(1, 1, 1));
		body->setLinearFactor(btVector3(1, 1, 1));
		body->applyTorqueImpulse(impulsevec);
		body->setDamping(0.9, 0.3);
		body->setRestitution(0.3f);
		//body->setFriction(0.4 * (i + 1));//come back to this
        body->forceActivationState(ISLAND_SLEEPING);
		body->setWorldTransform(startTransform);

		// 6. Add created rigid body to the world
		body->setUserIndex(-1);
		m_dynamicsWorld->addRigidBody(body);
		return body;
	}
    return NULL;
}
//void setcbfrommodel(dynent *d, const char *mdl)//set collision box
//{
//  	if (m_dynamicsWorld)
//	{
//
//    model *m = loadmodel(mdl);
//    if(!m) return;
//    vec center, radius;
//    m->collisionbox(center, radius);
//
//    //if(m->collide != COLLIDE_ELLIPSE) d->collidetype = COLLIDE_OBB;
//    d->xradius   = radius.x + fabs(center.x);
//    d->yradius   = radius.y + fabs(center.y);
//    //d->radius    = max(d->xradius, d->yradius);//sqrtf(d->xradius*d->xradius + d->yradius*d->yradius);
//    //d->eyeheight = (center.z-radius.z) + radius.z*2*m->eyeheight;
//    //d->aboveeye  = radius.z*2*(1.0f-m->eyeheight);    
//    ///scaling of the objects (0.1 = 20 centimeter boxes )
//    #define SCALING 0.058825//1//we will work this out in a bit
//
//    // 1. Create collision shape
//    // Re-using the same collision shape is better for memory usage and performance
//    //btCollisionShape* colShape = new btBoxShape(btVector3(SCALING * 1.2f, SCALING * 1.2f, SCALING * 1.2f));
//	btCollisionShape* colShape = new btBoxShape(btVector3(SCALING * (radius.x + fabs(center.x)), SCALING * ( fabs(center.z)) , SCALING * (radius.y + fabs(center.y))));
//    // 2. Set some properties of collision shape
//    colShape->setMargin(btScalar(0.04f));
//
//    //rigidbody is dynamic if and only if mass is non zero, otherwise static
//    btScalar mass(4.f);
//    btVector3 localInertia(0, 0, 1);
//
//    colShape->calculateLocalInertia(mass, localInertia);
//
//    // 3. Create motions state for rigid body
//    btDefaultMotionState* myMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),btVector3(0.0f, 0.0f, 0.0f)));
//
//    // 4. Create rigid body
//    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
//    rbInfo.m_angularSleepingThreshold = btScalar(0.0f);
//    rbInfo.m_linearSleepingThreshold = btScalar(0.0f);
//
//    btRigidBody* body = new btRigidBody(rbInfo);
//
//    // 5. Set properties of the rigid body
//    //d.feetpos().
//    btTransform startTransform;
//    startTransform.setIdentity();
//    
//    //startTransform.setOrigin(btVector3(30,50,30));//(float(d->o.x),float(d->o.y),float(d->o.z)));*-0.058825
//    //startTransform.setOrigin(btVector3(d->o.x*0.058825,d->o.z*0.058825,d->o.y*0.058825)); //have to scale location
//    startTransform.setOrigin(btVector3(d->feetpos().x*0.058825,(d->feetpos().z + (radius.z * 10))*0.058825,d->feetpos().y*0.058825)); //have to scale location
//    //conoutf("world pos = %f,%f,%f\n",float(d->feetpos().y),float(d->feetpos().z),float(d->feetpos().x));
//    body->setCenterOfMassTransform(startTransform);
//    body->setAngularFactor(btVector3(1, 1, 1));
//    body->setLinearFactor(btVector3(1, 1, 1));
//    body->setDamping(0.9, 0.3);
//    body->setRestitution(0.3f);
//    //body->setFriction(0.4 * (i + 1));//come back to this
//    body->forceActivationState(DISABLE_DEACTIVATION);
//    body->setWorldTransform(startTransform);
//
//    // 6. Add created rigid body to the world
//    body->setUserIndex(-1);
//    m_dynamicsWorld->addRigidBody(body);
//    conoutf("RigidBody added");
//	}
//}
/*
void mapcfgname()//this might be handy for bullet serialization eg: mapname.bullet//
{
    const char *mname = game::getclientmap();
    if(!*mname) mname = "untitled";

    defformatstring(cfgname, "media/map/%s.cfg", mname);
    path(cfgname);
    result(cfgname);
}
*/
