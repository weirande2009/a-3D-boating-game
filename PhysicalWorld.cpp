#include "PhysicalWorld.h"

Object_info::Object_info(float _objMass, glm::vec3 _objPosition, glm::vec3 _objScaling)
{
	objMass = _objMass;
	objPosition = glm_vec3_To_btVector3(_objPosition);
	objScaling = glm_vec3_To_btVector3(_objScaling);
	objEularZYX = btVector3(0.0f, 0.0f, 0.0f);
	objRestitution = 0.3f;
	objFriction = 0.5f;
	objRollFriction = 0.5f;
	objCentralImpulse = btVector3(0.0f, 0.0f, 0.0f);
}

Object_info::Object_info()
{
	objMass = 1.0f;
	objPosition = btVector3(0.0f, 0.0f, 0.0f);
	objScaling = btVector3(1.0f, 1.0f, 1.0f);;
	objEularZYX = btVector3(0.0f, 0.0f, 0.0f);
	objRestitution = 0.3f;
	objFriction = 0.5f;
	objRollFriction = 0.5f;
	objCentralImpulse = btVector3(0.0f, 0.0f, 0.0f);
}

void Object_info::setMass(float _objMass)
{
	objMass = _objMass;
}

void Object_info::setHeight(float _objHeight)
{
	objHeight = _objHeight;
}

void Object_info::setPosition(glm::vec3 _objPosition)
{
	objPosition = glm_vec3_To_btVector3(_objPosition);
}

void Object_info::setRotation(glm::vec3 _objEularZYX)
{
	objEularZYX = glm_vec3_To_btVector3(_objEularZYX);
}

void Object_info::setScaling(glm::vec3 _objScaling)
{
	objScaling = glm_vec3_To_btVector3(_objScaling);
}

void Object_info::setRestitution(float _objRestitution)
{
	objRestitution = _objRestitution;
}

void Object_info::setFriction(float _objFriction)
{
	objFriction = _objFriction;
}

void Object_info::setRollFriction(float _objRollFriction)
{
	objRollFriction = _objRollFriction;
}

void Object_info::setCentralImpulse(glm::vec3 _objCentralImpulse)
{
	objCentralImpulse = glm_vec3_To_btVector3(_objCentralImpulse);
}





PhysicalWorld::PhysicalWorld()
{
	/* 初始化物理引擎 */
	// create the collision configuration
	m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
	// create the dispatcher
	m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
	btGImpactCollisionAlgorithm::registerAlgorithm(m_pDispatcher);
	// create the broadphase
	m_pBroadphase = new btDbvtBroadphase();
	// create the constraint solver
	m_pSolver = new btSequentialImpulseConstraintSolver();

	/* 创建世界 */
	// create the world
	m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);

	/* 设置重力 */
	m_pWorld->setGravity(btVector3(0, gravity, 0));
}

PhysicalWorld::~PhysicalWorld()
{
	//cleanup in the reverse order of creation/initialization

	//-----cleanup_start-----

	//remove the rigidbodies from the dynamics world and delete them
	for (int i = m_pWorld->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = m_pWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_pWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = m_pWorld->getNumCollisionObjects() - 1; j >= 0; j--)
	{
		btCollisionObject* obj = m_pWorld->getCollisionObjectArray()[j];
		removeObject(obj);
	}

	delete m_pWorld;
	delete m_pSolver;
	delete m_pBroadphase;
	delete m_pDispatcher;
	delete m_pCollisionConfiguration;

}

/* 不使用 */
void PhysicalWorld::CreatePlane()
{
	//std::vector<btVector3> points;
	//points.push_back(btVector3(30.0f, 0.0f, 30.0f));
	//points.push_back(btVector3(30.0f, 0.0f, -30.0f));
	//points.push_back(btVector3(-30.0f, 0.0f, -30.0f));
	//points.push_back(btVector3(-30.0f, 0.0f, 30.0f));
	//CreateStaticObject(points, btVector3(0.0f, 0.0f, 0.0f));
}

void PhysicalWorld::CreateCube(btVector3 shape, btVector3 position)
{
	/* 创建正方体，边长为2 */
	//create a dynamic rigidbody
	btCollisionShape* colShape = new btBoxShape(btVector3(shape.x()/2, shape.y()/2, shape.z()/2));

	/* 设置旋转和平移结构 */
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(position);

	/* 设置质量 */
	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	/* 计算惯性 */
	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	/* 设置冲量，即运动速度 */
	body->applyCentralImpulse(btVector3(0, 0, 0));

	m_pWorld->addRigidBody(body);

	/* 初始化物体位置信息 */
	objPos.push_back(position);
	objEulerZYX.push_back(btVector3(0, 0, 0));
}

void PhysicalWorld::CreateSphere(btScalar r, btVector3 position, btVector3 CentralImpulse)
{
	/* 创建正方体，边长为2 */
//create a dynamic rigidbody
	btCollisionShape* colShape = new btSphereShape(r);

	/* 设置旋转和平移结构 */
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(position);

	/* 设置质量 */
	btScalar mass(1.f);

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	/* 计算惯性 */
	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		colShape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	/* 设置碰撞反弹系数  默认为0 */
	body->setRestitution(btScalar(0.8f));
	/* 设置摩擦力 */
	body->setFriction(btScalar(2));
	body->setRollingFriction(btScalar(0.005f));
	/* 设置冲量，即运动速度 */
	body->applyCentralImpulse(CentralImpulse);

	m_pWorld->addRigidBody(body);

	/* 初始化物体位置信息 */
	objPos.push_back(position);
	objEulerZYX.push_back(btVector3(0, 0, 0));
}

void PhysicalWorld::CreateStaticObject(const Object_info & obj_info)
{
	btTriangleMesh *triangle_mesh = new btTriangleMesh();
	for (int i = 0; i < obj_info.objPoints.size(); i += 3) {
		btVector3 vertex_1 = obj_info.objPoints[i];
		btVector3 vertex_2 = obj_info.objPoints[i + 1];
		btVector3 vertex_3 = obj_info.objPoints[i + 2];

		triangle_mesh->addTriangle(vertex_1, vertex_2, vertex_3);
	}
	btBvhTriangleMeshShape *triangle_mesh_shape = new btBvhTriangleMeshShape(triangle_mesh, true);
	btBvhTriangleMeshShape *collisionShape = new btBvhTriangleMeshShape(*triangle_mesh_shape);

	/* 设置缩放 */
	collisionShape->setLocalScaling(obj_info.objScaling);

	/* 设置旋转和平移结构 */
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(obj_info.objPosition);
	startTransform.setRotation(btQuaternion(obj_info.objEularZYX.x(), obj_info.objEularZYX.y(), obj_info.objEularZYX.z()));


	/* 物体质量 */
	btScalar mass(obj_info.objMass);
	bool isDynamic = (mass != 0.0f);

	/* 计算物体惯性 */
	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		collisionShape->calculateLocalInertia(mass, localInertia);

		/* 初始化物体位置信息 */
		/* 添加碰撞体信息 */
		objPos.push_back(obj_info.objPosition);
		objEulerZYX.push_back(obj_info.objEularZYX);
	}

	btDefaultMotionState* groundMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, groundMotionState, collisionShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//body->setActivationState(4);


	/* 设置碰撞反弹系数 */
	body->setRestitution(obj_info.objRestitution);
	/* 设置滑动摩擦力系数 */
	body->setFriction(obj_info.objFriction);
	/* 设置滚动摩擦力系数 */
	body->setFriction(obj_info.objRollFriction);

	m_pWorld->addRigidBody(body);

	static_obj_num++;
}

int PhysicalWorld::CreateDynamicObject(const Object_info & obj_info)
{
	/* 刚体是动态的如果且仅当质量为非零时，否则是静止的 */
	/* 添加点 */
	btConvexHullShape * unoptimized_hull = new btConvexHullShape();
	for (int i = 0; i < obj_info.objPoints.size(); ++i)
		unoptimized_hull->addPoint(obj_info.objPoints[i]);

	btShapeHull *hull_optimizer = new btShapeHull(unoptimized_hull);

	btScalar margin(unoptimized_hull->getMargin());
	hull_optimizer->buildHull(margin);

	btConvexHullShape* hull = new btConvexHullShape(
		(btScalar*)hull_optimizer->getVertexPointer(),
		hull_optimizer->numVertices());

	btCollisionShape* collisionShape = new btConvexHullShape(*hull);

	/* 设置缩放 */
	collisionShape->setLocalScaling(obj_info.objScaling);


	/* 设置旋转和平移结构 */
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(obj_info.objPosition);
	startTransform.setRotation(btQuaternion(obj_info.objEularZYX.x(), obj_info.objEularZYX.y(), obj_info.objEularZYX.z()));
	
	/* 物体质量 */
	btScalar mass(obj_info.objMass);
	bool isDynamic = (mass != 0.0f);

	/* 计算物体惯性 */
	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		collisionShape->calculateLocalInertia(mass, localInertia);

		/* 初始化物体位置信息 */
		/* 添加碰撞体信息 */
		objPos.push_back(obj_info.objPosition);
		objEulerZYX.push_back(obj_info.objEularZYX);
	}

	btDefaultMotionState* MotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, MotionState, collisionShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//body->setActivationState(4);

	/* 设置碰撞反弹系数  默认为 0*/
	body->setRestitution(obj_info.objRestitution);
	/* 设置滑动摩擦力系数  默认为 0*/
	body->setFriction(obj_info.objFriction);
	/* 设置滚动摩擦力系数  默认为 0*/
	body->setFriction(obj_info.objRollFriction);
	/* 设置冲量，即运动速度 */
	body->applyCentralImpulse(obj_info.objCentralImpulse);
	/* 加入刚体 */
	m_pWorld->addRigidBody(body);

	/* 返回物体编号 */
	if (isDynamic)
		return objPos.size() - 1;
	else
		return -1;
}

/* 不使用，暂时有问题，抛弃 */
int PhysicalWorld::CreateDynamicObject2(const Object_info & obj_info) 
{
	int triangle_num = obj_info.objPoints.size() / 3;
	int vertex_num = obj_info.objPoints.size();
	std::vector<int> index;
	std::vector<btScalar> vertex;
	for (int i = 0; i < vertex_num; ++i) {
		vertex.push_back(obj_info.objPoints[i].x());
		vertex.push_back(obj_info.objPoints[i].y());
		vertex.push_back(obj_info.objPoints[i].z());

		index.push_back(i);
	}
	btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(triangle_num, //片面数据
		&index[0],
		3 * sizeof(int),
		vertex_num, &vertex[0], sizeof(btScalar) * 3);

	btGImpactMeshShape * collisionShape = new btGImpactMeshShape(indexVertexArrays);
	collisionShape->updateBound();


	/* 设置缩放 */
	collisionShape->setLocalScaling(obj_info.objScaling);

	/* 设置旋转和平移结构 */
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(obj_info.objPosition);
	startTransform.setRotation(btQuaternion(obj_info.objEularZYX.x(), obj_info.objEularZYX.y(), obj_info.objEularZYX.z()));


	/* 物体质量 */
	btScalar mass(obj_info.objMass);
	bool isDynamic = (mass != 0.0f);

	/* 计算物体惯性 */
	btVector3 localInertia(0, 0, 0);
	if (isDynamic) {
		collisionShape->calculateLocalInertia(mass, localInertia);

		/* 初始化物体位置信息 */
		/* 添加碰撞体信息 */
		objPos.push_back(obj_info.objPosition);
		objEulerZYX.push_back(obj_info.objEularZYX);
	}

	btDefaultMotionState* MotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, MotionState, collisionShape, localInertia);
	btRigidBody* body = new btRigidBody(rbInfo);

	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_pWorld->getDispatcher());
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	/* 设置碰撞反弹系数  默认为 0*/
	body->setRestitution(obj_info.objRestitution);
	/* 设置滑动摩擦力系数  默认为 0*/
	body->setFriction(obj_info.objFriction);
	/* 设置滚动摩擦力系数  默认为 0*/
	body->setFriction(obj_info.objRollFriction);
	/* 设置冲量，即运动速度 */
	body->applyCentralImpulse(obj_info.objCentralImpulse);
	/* 加入刚体 */
	m_pWorld->addRigidBody(body);

	if (isDynamic)
		return objPos.size() - 1;
	else
		return -1;
}


void PhysicalWorld::stepSimulation(std::vector<int> &Delete_obj_index)
{
	/* 模拟一步 */
	m_pWorld->stepSimulation(1.f / 60.f, 10);

	/* 动态物体索引 */
	int index = objPos.size();

	/* 更新所有物体的位置信息，除了地表 */
	for (int j = m_pWorld->getNumCollisionObjects() - 1; j >= 0; j--)
	{
		btCollisionObject* obj = m_pWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		if (body && body->getMotionState())
		{
			body->getMotionState()->getWorldTransform(trans);
		}
		else
		{
			trans = obj->getWorldTransform();
		}

		
		btVector3 pos = trans.getOrigin();
		btScalar yawZ, pitchY, rollX;
		btQuaternion rot = trans.getRotation();
		rot.getEulerZYX(yawZ, pitchY, rollX);

		if (body->getMass() != 0.0f) {
			index -= 1;

			/* 更新物体位置 */
			/* 更新物体欧拉角 */
			objPos[index] = pos;
			btVector3 EulerZYZ(yawZ, pitchY, rollX);
			objEulerZYX[index] = EulerZYZ;

			/* 如果小于规定的界限，则将刚体删除 */
			if (pos.z() >= Min_Z_Level) {
				///* 删除刚体 */
				//removeObject(obj);

				///* 删除对应的位置信息 */
				//objPos.erase(objPos.begin() + index);
				//objEulerZYX.erase(objEulerZYX.begin() + index);

				/* 将删除的物体标号传回（以0起始） */
				Delete_obj_index.push_back(index);
				
			}

			//if (index == 0)
			//	printf("world pos %d = %f,%f,%f\n",
			//		j, float(pos.x()), float(pos.y()), float(pos.z())
			//	);
		}

	}
}

void PhysicalWorld::removeObject(btCollisionObject* obj)
{
	btRigidBody* body = btRigidBody::upcast(obj);
	if (body && body->getMotionState())
	{
		delete body->getMotionState();
		delete body->getCollisionShape();
	}

	m_pWorld->removeCollisionObject(obj);
	delete obj;
}

btVector3 glm_vec3_To_btVector3(glm::vec3 vec3)
{
	return btVector3(vec3.x, vec3.y, vec3.z);
}

glm::vec3 btVector3_To_glm_vec3(btVector3 btvec3)
{
	return glm::vec3(btvec3.x(), btvec3.y(), btvec3.z());
}

std::vector<btVector3> vector_glm_vec3_To_vector_btVector3(std::vector<glm::vec3> vec3)
{
	std::vector<btVector3> btvec3;
	for (int i = 0; i<int(vec3.size()); i++) {
		btvec3.push_back(btVector3(vec3[i].x, vec3[i].y, vec3[i].z));
	}
	return btvec3;
}

/* 计算当前物体在水中受到的浮力、水的推力、水的阻力 */
void compute_forces(PhysicalWorld& world, Object_info objInfo)
{
	/* 动态物体索引 */
	int index = 0;
	for (int j = 0; j < world.m_pWorld->getNumCollisionObjects(); j++) {
		btCollisionObject* obj = world.m_pWorld->getCollisionObjectArray()[j];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		if (body && body->getMotionState()) {
			body->getMotionState()->getWorldTransform(trans);
		}
		else {
			trans = obj->getWorldTransform();
		}
		if (body->getMass() != 0.0f && index == objInfo.id) {
			btVector3 pos = trans.getOrigin();
			if (pos.x() > world.water_begin_x && pos.x() < world.water_end_x && pos.z() > world.water_begin_z && pos.z() < world.water_end_z) {
				//浮力
				btVector3 buoyancy_force(0.0f, 0.0f, 0.0f);
				//水的阻力
				btVector3 water_drag_force(0.0f, 0.0f, 0.0f);
				//水的推力
				btVector3 water_push_force(0.0f, 0.0f, 0.0f);
				//物体中点与水面的距离
				float distance_y = pos.y() - world.water_y;
				//物体的高度
				float object_h = objInfo.objHeight;
				//物体的单位体积
				float volumn_per_y = 5.0f / object_h;
				//水给物体的阻力系数
				btVector3 water_drag_coefficient(0.01f, 0.2f, 0.01f);
				//物体当前在三个轴上的速度
				btVector3 object_speed = body->getLinearVelocity();
				/* 根据物体与水的位置关系进行受力的计算 */
				if (abs(distance_y) < 0.5 * object_h) {//若此时物体进入水中但没有完全进入
					/* 计算水给物体的浮力 */
					buoyancy_force.setY(volumn_per_y * (-distance_y + 0.5 * object_h) * -world.gravity);
					/* 计算水给物体的阻力 */
					water_drag_force = water_drag_coefficient * -object_speed;
					/* 计算水给物体的推力 */
					if (body->getLinearVelocity().z() > world.water_speed_threshold) {
						water_push_force = world.water_push_force;
					}
					else {
						water_push_force = btVector3(0.0f, 0.0f, 0.0f);
					}
				}
				else if (distance_y <= -0.5 * object_h) {//若此时物体完全进入水中
					/* 计算水给物体的浮力 */
					buoyancy_force.setY(volumn_per_y * object_h * -world.gravity);
					/* 计算水给物体的阻力 */
					water_drag_force = water_drag_coefficient * -object_speed;
					/* 计算水给物体的推力 */
					if (body->getLinearVelocity().z() > world.water_speed_threshold) {
						water_push_force = world.water_push_force;
					}
					else {
						water_push_force = btVector3(0.0f, 0.0f, 0.0f);
					}
				}
				else {//若此时物体未进入水中
					/* 计算水给物体的浮力 */
					buoyancy_force.setY(0.0f);
					/* 计算水给物体的阻力 */
					water_drag_force = btVector3(0.0f, 0.0f, 0.0f);
					/* 计算水给物体的推力 */
					water_push_force = btVector3(0.0f, 0.0f, 0.0f);
				}


				//将上述的力相加
				body->applyCentralForce(buoyancy_force + water_drag_force + water_push_force);
			}
			index++;
		}
		else if (body->getMass() != 0.0f)
			index++;
	}
}


/* 将越出边界的水果直接传回起点 */
void moveObjToStart(PhysicalWorld& world, std::vector<int> deleteIndex)
{
	for (int i = 0; i<int(deleteIndex.size()); i++) {
		btCollisionObject* obj = world.m_pWorld->getCollisionObjectArray()[deleteIndex[i] + world.static_obj_num];
		btRigidBody* body = btRigidBody::upcast(obj);
		btTransform trans;
		if (body && body->getMotionState()) {
			body->getMotionState()->getWorldTransform(trans);
		}
		else {
			trans = obj->getWorldTransform();
		}
		btVector3 start_position = trans.getOrigin();
		start_position.setZ(-200.0f);
		start_position.setY(5.0f);
		start_position.setX(0.0f);
		//每个新生成的水果从天下落下，在有多个水果同时被消除时，水果的高度递增
		start_position.setY(start_position.y() * (i + 1));
		trans.setOrigin(start_position);
		btMotionState* motionState = body->getMotionState();
		motionState->setWorldTransform(trans);
		body->setMotionState(motionState);
		body->setLinearVelocity(btVector3(0.0f, 0.0f, 10.0f));
	}
}
