#pragma once
#include "include/learnopengl/model.h"
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <time.h>
class Fruit {
protected:
	Model _model;//模型
	//glm::vec3 _position;//当前位置
	//glm::vec3 _eularZYX;//旋转矩阵
	//glm::vec3 _scaling;//缩放
//	float speed;
public:
	Fruit(const Model& model): 
		_model(model)
	{
	};

	/* 渲染 */
	void render(Shader& shader, Camera& camera)
	{
		// 视图转换
		glm::mat4 viewMatrix = camera.GetViewMatrix();
		shader.setMat4("view", viewMatrix);
		// 模型转换
		glm::vec3 _position = getPos();
		glm::vec3 _eularZYX = getEular();
		glm::vec3 _scaling = getScaling();

		glm::mat4 modelMatrix = glm::mat4(1.0f);
		modelMatrix = glm::translate(modelMatrix, _position);
		modelMatrix = glm::rotate(modelMatrix, _eularZYX.x, glm::vec3(0.0f, 0.0f, 1.0f));
		modelMatrix = glm::rotate(modelMatrix, _eularZYX.y, glm::vec3(0.0f, 1.0f, 0.0f));
		modelMatrix = glm::rotate(modelMatrix, _eularZYX.z, glm::vec3(1.0f, 0.0f, 0.0f));
		modelMatrix = glm::scale(modelMatrix, _scaling);
		shader.setMat4("model", modelMatrix);
		// 投影转换
		glm::mat4 projMatrix = camera.GetProjMatrix((float)SCR_WIDTH / (float)SCR_HEIGHT);
		shader.setMat4("projection", projMatrix);

		_model.Draw(shader);
	}

	/* 获取物体位置 */
	glm::vec3 getPos() {
		return btVector3_To_glm_vec3(_model.obj_info.objPosition);
	}

	/* 获取欧拉角 */
	glm::vec3 getEular() {
		return btVector3_To_glm_vec3(_model.obj_info.objEularZYX);
	}

	/* 获取缩放 */
	glm::vec3 getScaling() {
		return btVector3_To_glm_vec3(_model.obj_info.objScaling);
	}

	/* 获取物体物理世界序号 */
	int getObjectId()
	{
		return _model.obj_info.id;
	}

	Object_info getObjectInfo()
	{
		return _model.obj_info;
	}

	/* 不再使用 */
	//void move(const glm::vec3& direction, float deltaTime) {
	//	_position += deltaTime * speed * direction;
	//}

	/* 设置物体位置 */
	void setPos(const glm::vec3& pos) {
		_model.obj_info.setPosition(pos);
	}

	/* 设置物体欧拉角，初始化和更新 */
	void setEular(const glm::vec3& eular) {
		_model.obj_info.setRotation(eular);
	}

	/* 设置缩放，不建议使用，物理世界中会出问题 */
	void setScaling(const glm::vec3& scaling) {
		_model.obj_info.setScaling(scaling);
	}

	/* 设置物体在物理世界中的编号，不要轻易更改 */
	void setObjectId(const int &id) {
		_model.obj_info.id = id;
	}

	/* 物理世界创建物体 */
	void createFruitInPhysicalWorld(PhysicalWorld &world) {
		_model.CreateObjectInPhysicalWorld(world);
	}

	/* 更新物体位置和欧拉角 */
	void updateObjectInfo(PhysicalWorld &world)
	{
		glm::vec3 glm_pos = btVector3_To_glm_vec3(world.objPos[_model.obj_info.id]);
		glm::vec3 glm_eular = btVector3_To_glm_vec3(world.objEulerZYX[_model.obj_info.id]);

		//printf("position %d = %f, %f, %f\n", _model.obj_info.id, glm_pos.x, glm_pos.y, glm_pos.z);
		/* 更新物体位置和欧拉角 */
		setPos(glm_pos);
		setEular(glm_eular);
	}
};

class FruitManager {
protected:
	std::vector<std::shared_ptr<Fruit>> fruits;
	float generate_y = 0; // 生成的y轴位置（纵向）
	std::pair<float, float> generate_z = {-200, -100};// 生成的z轴位置（前后）
	std::pair<float, float> generate_x = { -10, -2 }; // 左右
	float distroy_z = 50.0f; // z值超过该位置则销毁
	int generateSpeed = 0; // 每秒生成速度
	float moveSpeed = 20.0f; // 移动速度
	glm::vec3 moveDirection = {0, 0, 1};//移动方向
	int maxCount = 10;

	std::vector<std::string> fruitPaths;

	float random() {
		
		return rand() / float(RAND_MAX);
	}
public:
	FruitManager(int generate_speed, float move_speed) {
		srand((unsigned)time(NULL));
		generateSpeed = generate_speed;
		moveSpeed = move_speed;
		for (int i = 1; i <= 25; i++) {
			std::string temp = "asset/models/obj/fruit/";
			temp += to_string(i);
			temp += ".obj";
			fruitPaths.push_back(FileSystem::getPath(temp));
		};
	};

	/* 更新位置信息 */
	void update(PhysicalWorld &world) {
		//printf("%d\n", fruits.size());

		for (int i = 0; i < fruits.size(); i++){
			compute_forces(world, fruits[i]->getObjectInfo());
			fruits[i]->updateObjectInfo(world);
		}
		//const size_t len = fruits.size();
		//int destroy_count = 0;
		//for (int i = 0; i < len - destroy_count; i++) {
		//	float zz = fruits[i]->getPos()[2];
		//	if (zz > distroy_z) { // 超出则销毁
		//		swap(fruits[i], fruits[len - destroy_count - 1]);// 交换
		//		destroy_count++;
		//	}
		//}
		//fruits.erase(fruits.end() - destroy_count, fruits.end());
		//if (fruits.size() != len - destroy_count)
		//	return;
		//// 检查是否超出最大数量，超出则不用生成
		//if (fruits.size() >= maxCount)
		//	return;
		//// 生成
		//int n = 0;
		//float p = random();
		//if (p < (generateSpeed * deltaTime))// 生成数量的期望
		//	n = 1;
		//using std::cout;
		//if (n == 1) {
		//	int rank = rand() % fruitPaths.size();
		//	Model model(fruitPaths[rank], Object_info());
		//	float y = generate_y;
		//	float x = generate_x.first;
		//	x += random() * (generate_x.second - generate_x.first);
		//	float z = generate_z.first;
		//	z += random() * (generate_z.second - generate_z.first);
		//	float speed = moveSpeed - 30 + random() * 60;
		//	cout << rank << " " << x << "\n";
		//	std::shared_ptr<Fruit> fp = make_shared<Fruit>(model);
		//	fruits.push_back(fp);
		//}
		return;
	}

	/* 销毁 */
	void deleteFruit(std::vector<int> &deleteObjIndex)
	{
		// 销毁
		int delete_obj_num = deleteObjIndex.size();

		/* 由大到小的顺序遍历 */
		for (int index = 0; index < delete_obj_num; ++index) {
			for (int fruit_no = fruits.size() - 1; fruit_no >= 0; --fruit_no) {
				int obj_id = fruits[fruit_no]->getObjectId();
				if (obj_id > deleteObjIndex[index])
					fruits[fruit_no]->setObjectId(obj_id - 1);
				else
					fruits.erase(fruits.begin() + fruit_no);
			}

			for (int j = index + 1; j < delete_obj_num; ++j)
				if (deleteObjIndex[j] > deleteObjIndex[index])
					deleteObjIndex[j] -= 1;
		}
	}

	/* 生成 */
	void createFruit(PhysicalWorld &world, float deltaTime)
	{
		// 生成
		/* 检查是否超出最大数量，超出则不用生成 */
		if (fruits.size() >= maxCount)
			return;
		// 生成
		int n = 0;
		float p = random();
		if (p < (generateSpeed * deltaTime))// 生成数量的期望
			n = 1;
		using std::cout;
		if (n == 1) {
			/* 随机生成水果 */
			int rank = rand() % fruitPaths.size();
			Model model(fruitPaths[rank], Object_info());
			/* 随机位置 */
			float y = generate_y;
			float x = generate_x.first;
			x += random() * (generate_x.second - generate_x.first);
			float z = generate_z.first;
			z += random() * (generate_z.second - generate_z.first);

			model.obj_info.setPosition(glm::vec3(x, y, z));
			model.obj_info.setCentralImpulse(moveDirection * moveSpeed);//设置初速度

			/* 物理世界中创建物体 */
			model.CreateObjectInPhysicalWorld(world);
			//			cout << rank << " " << x << "\n";
			std::shared_ptr<Fruit> fp = make_shared<Fruit>(model);
			fruits.push_back(fp);
		}
	}

	void render(Shader& shader, Camera& camera) {
		for (auto& f : fruits) {
			f->render(shader, camera);
		}
	}
};