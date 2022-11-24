#version 330 core
layout(location = 6) in vec3 aPos;

uniform mat4 modelMat;
uniform mat4 viewMat;
uniform mat4 projMat;

out vec3 FragPos;

void main(){
	gl_Position = projMat * viewMat * modelMat * vec4(aPos.xyz, 1.0);
	FragPos = (modelMat * vec4(aPos.xyz, 1.0f)).xyz;
}