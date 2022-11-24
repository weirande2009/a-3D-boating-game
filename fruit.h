#pragma once
#include "include/learnopengl/model.h"
#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <time.h>
class Fruit {
protected:
	Model _model;//ģ��
	//glm::vec3 _position;//��ǰλ��
	//glm::vec3 _eularZYX;//��ת����
	//glm::vec3 _scaling;//����
//	float speed;
public:
	Fruit(const Model& model): 
		_model(model)
	{
	};

	/* ��Ⱦ */
	void render(Shader& shader, Camera& camera)
	{
		// ��ͼת��
		glm::mat4 viewMatrix = camera.GetViewMatrix();
		shader.setMat4("view", viewMatrix);
		// ģ��ת��
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
		// ͶӰת��
		glm::mat4 projMatrix = camera.GetProjMatrix((float)SCR_WIDTH / (float)SCR_HEIGHT);
		shader.setMat4("projection", projMatrix);

		_model.Draw(shader);
	}

	/* ��ȡ����λ�� */
	glm::vec3 getPos() {
		return btVector3_To_glm_vec3(_model.obj_info.objPosition);
	}

	/* ��ȡŷ���� */
	glm::vec3 getEular() {
		return btVector3_To_glm_vec3(_model.obj_info.objEularZYX);
	}

	/* ��ȡ���� */
	glm::vec3 getScaling() {
		return btVector3_To_glm_vec3(_model.obj_info.objScaling);
	}

	/* ��ȡ��������������� */
	int getObjectId()
	{
		return _model.obj_info.id;
	}

	Object_info getObjectInfo()
	{
		return _model.obj_info;
	}

	/* ����ʹ�� */
	//void move(const glm::vec3& direction, float deltaTime) {
	//	_position += deltaTime * speed * direction;
	//}

	/* ��������λ�� */
	void setPos(const glm::vec3& pos) {
		_model.obj_info.setPosition(pos);
	}

	/* ��������ŷ���ǣ���ʼ���͸��� */
	void setEular(const glm::vec3& eular) {
		_model.obj_info.setRotation(eular);
	}

	/* �������ţ�������ʹ�ã����������л������ */
	void setScaling(const glm::vec3& scaling) {
		_model.obj_info.setScaling(scaling);
	}

	/* �������������������еı�ţ���Ҫ���׸��� */
	void setObjectId(const int &id) {
		_model.obj_info.id = id;
	}

	/* �������紴������ */
	void createFruitInPhysicalWorld(PhysicalWorld &world) {
		_model.CreateObjectInPhysicalWorld(world);
	}

	/* ��������λ�ú�ŷ���� */
	void updateObjectInfo(PhysicalWorld &world)
	{
		glm::vec3 glm_pos = btVector3_To_glm_vec3(world.objPos[_model.obj_info.id]);
		glm::vec3 glm_eular = btVector3_To_glm_vec3(world.objEulerZYX[_model.obj_info.id]);

		//printf("position %d = %f, %f, %f\n", _model.obj_info.id, glm_pos.x, glm_pos.y, glm_pos.z);
		/* ��������λ�ú�ŷ���� */
		setPos(glm_pos);
		setEular(glm_eular);
	}
};

class FruitManager {
protected:
	std::vector<std::shared_ptr<Fruit>> fruits;
	float generate_y = 0; // ���ɵ�y��λ�ã�����
	std::pair<float, float> generate_z = {-200, -100};// ���ɵ�z��λ�ã�ǰ��
	std::pair<float, float> generate_x = { -10, -2 }; // ����
	float distroy_z = 50.0f; // zֵ������λ��������
	int generateSpeed = 0; // ÿ�������ٶ�
	float moveSpeed = 20.0f; // �ƶ��ٶ�
	glm::vec3 moveDirection = {0, 0, 1};//�ƶ�����
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

	/* ����λ����Ϣ */
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
		//	if (zz > distroy_z) { // ����������
		//		swap(fruits[i], fruits[len - destroy_count - 1]);// ����
		//		destroy_count++;
		//	}
		//}
		//fruits.erase(fruits.end() - destroy_count, fruits.end());
		//if (fruits.size() != len - destroy_count)
		//	return;
		//// ����Ƿ񳬳����������������������
		//if (fruits.size() >= maxCount)
		//	return;
		//// ����
		//int n = 0;
		//float p = random();
		//if (p < (generateSpeed * deltaTime))// ��������������
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

	/* ���� */
	void deleteFruit(std::vector<int> &deleteObjIndex)
	{
		// ����
		int delete_obj_num = deleteObjIndex.size();

		/* �ɴ�С��˳����� */
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

	/* ���� */
	void createFruit(PhysicalWorld &world, float deltaTime)
	{
		// ����
		/* ����Ƿ񳬳���������������������� */
		if (fruits.size() >= maxCount)
			return;
		// ����
		int n = 0;
		float p = random();
		if (p < (generateSpeed * deltaTime))// ��������������
			n = 1;
		using std::cout;
		if (n == 1) {
			/* �������ˮ�� */
			int rank = rand() % fruitPaths.size();
			Model model(fruitPaths[rank], Object_info());
			/* ���λ�� */
			float y = generate_y;
			float x = generate_x.first;
			x += random() * (generate_x.second - generate_x.first);
			float z = generate_z.first;
			z += random() * (generate_z.second - generate_z.first);

			model.obj_info.setPosition(glm::vec3(x, y, z));
			model.obj_info.setCentralImpulse(moveDirection * moveSpeed);//���ó��ٶ�

			/* ���������д������� */
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