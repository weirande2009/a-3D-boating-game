#pragma once
#include <glm/glm.hpp>
// ���ڳߴ�
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;

// ��Ⱦ��Ӱʱ�Ĵ��ڷֱ��ʣ���Ӱ����Ӱ�ľ�ݱ������
const unsigned int SHADOW_WIDTH = 1024 * 10;
const unsigned int SHADOW_HEIGHT = 1024 * 10;

// �Ƿ�Ϊ�߿�ͼģʽ
bool isPolygonMode = false;

// ��������ϵY�ᵥλ����
glm::vec3 WORLD_UP(0.0f, 1.0f, 0.0f);