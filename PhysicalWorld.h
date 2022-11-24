#pragma once
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <glm/glm.hpp>
#include <time.h>
#include <vector>

#define Min_Z_Level 10

class PhysicalWorld;
class Object_info;

btVector3 glm_vec3_To_btVector3(glm::vec3 vec3);

glm::vec3 btVector3_To_glm_vec3(btVector3 btvec3);

std::vector<btVector3> vector_glm_vec3_To_vector_btVector3(std::vector<glm::vec3> vec3);

void compute_forces(PhysicalWorld &world, Object_info objInfo);

void moveObjToStart(PhysicalWorld &world, std::vector<int> deleteIndex);

class Object_info
{
public:
	btScalar objMass;
	btScalar objHeight;
	btVector3 objPosition;
	btVector3 objEularZYX;
	btVector3 objScaling;
	btScalar objRestitution;	// 反弹系数
	btScalar objFriction;		// 摩擦系数
	btScalar objRollFriction;	// 滚动摩擦系数
	btVector3 objCentralImpulse;	// 冲量，速度
public:
	int id;
	std::vector<btVector3> objPoints;

	Object_info(float _objMass, glm::vec3 _objPosition, glm::vec3 _objScaling);
	Object_info();
	void setMass(float _objMass);
	void setHeight(float _objHeight);
	void setPosition(glm::vec3 _objPosition);
	void setRotation(glm::vec3 _objEularZYX);
	void setScaling(glm::vec3 _objScaling);
	void setRestitution(float _objRestitution);
	void setFriction(float _objFriction);
	void setRollFriction(float _objRollFriction);
	void setCentralImpulse(glm::vec3 _objCentralImpulse);
};


class PhysicalWorld
{
public:
	int static_obj_num = 0;

	std::vector<btVector3> objPos;
	std::vector<btVector3> objEulerZYX;

		//物理世界的重力系数g
	btScalar gravity = -1.81;

    //水面高度
	btScalar water_y = 0.0f;
	//水面范围
	btScalar water_begin_x = -50.0f;
	btScalar water_end_x = 50.0f;
	btScalar water_begin_z = -700.0f;
	btScalar water_end_z = 100.0f;
	//水给物体的推力的速度阈值
	btScalar water_speed_threshold = -1.0f;
	//水给物体的推力
	btVector3 water_push_force = btVector3(0.0f, 0.0f, 0.1f);



	// core Bullet components
	btBroadphaseInterface* m_pBroadphase;
	btCollisionConfiguration* m_pCollisionConfiguration;
	btCollisionDispatcher* m_pDispatcher;
	btConstraintSolver* m_pSolver;
	btDynamicsWorld* m_pWorld;

	PhysicalWorld();
	~PhysicalWorld();

	void CreatePlane();
	void CreateCube(btVector3 shape = btVector3(1, 1, 1), btVector3 position = btVector3(0, 10, 0));
	void CreateSphere(btScalar r = 1.0f, btVector3 position = btVector3(0, 10, 0), btVector3 CentralImpulse = btVector3(0.0f, 0.0f, 0.0f));
	void CreateStaticObject(const Object_info &obj_info);
	int CreateDynamicObject(const Object_info &obj_info);
	int CreateDynamicObject2(const Object_info & obj_info);

	void stepSimulation(std::vector<int> &Delete_obj_index);
	void removeObject(btCollisionObject* obj);
};

