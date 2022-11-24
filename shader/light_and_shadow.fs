#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
    vec2 TexCoords;
    vec4 FragPosLightSpace;
    vec4 Ambient;
    vec4 Diffuse;
    vec4 Specular;
} fs_in;

uniform sampler2D diffuseTexture;
uniform sampler2D shadowMap;
uniform bool hasTex;
// ����
struct Material{
    vec4 ambient;
    vec4 diffuse;
    vec4 specular;
    float shininess;
};
uniform Material material;

// uniform vec3 lightPos;
struct Light
{
    vec3 position;
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
    vec3 direction;
};
uniform Light light;
uniform vec3 viewPos;

float ShadowCalculation(vec4 fragPosLightSpace)
{
    // ͸�ӳ���
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // ӳ�䵽[0,1]����
    projCoords = projCoords * 0.5 + 0.5;
    // �ӹ�ĽǶȵõ�ָ��ǰƬԪ��·���ϵ���������ֵ��ʹ��[0,1]����fragPosLight���꣩
    float closestDepth = texture(shadowMap, projCoords.xy).r; 
    // �ӹ�ĽǶȵõ���ǰƬԪ�����
    float currentDepth = projCoords.z;
    
    // ����ƫ��
    vec3 normal = normalize(fs_in.Normal);
    float bias = max(0.001 * (1.0 - dot(normal, light.direction)), 0.00001);
    
    // ��⵱ǰfrag�Ƿ�����Ӱ��
    // float shadow = currentDepth - bias > closestDepth ? 1.0 : 0.0;

    // PCF �Ż������Ӱ��Ե
    float shadow = 0.0;
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    for(int x = -1; x <= 1; ++x) {
        for(int y = -1; y <= 1; ++y) {
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth - bias > pcfDepth ? 1.0 : 0.0;
        }
    }
    shadow /= 9.0;
    
    // ��ƬԪ��ȳ���1.0ʱ����������Ӱ������Զ��ȫ����Ӱ������
    if(projCoords.z > 1.0)
        shadow = 0.0;
        
    return shadow;
}

void main()
{
    vec3 color;
    if (hasTex)
        color = texture(diffuseTexture, fs_in.TexCoords).rgb;
    else
        color = fs_in.Diffuse.rgb;
    vec3 normal = normalize(fs_in.Normal);
    vec3 lightColor = vec3(1, 1, 0.95); // �ƹ���ɫ

    // ambient ������
    vec3 ambient = 0.1 * color;
    
    // diffuse ������
    float diff = max(dot(light.direction, normal), 0.0);
    vec3 diffuse = diff * color * lightColor;  // ��ͼ��ɫԽ�������Խ��
    
    // specular ���淴��
    vec3 viewDir = normalize(viewPos - fs_in.FragPos);
    vec3 reflectDir = reflect(-light.direction, normal);
    
    float spec = 0.0;
    vec3 halfwayDir = normalize(light.direction + viewDir);
    spec = pow(max(dot(normal, halfwayDir), 0.0), 32.0);
    vec3 specular = spec * lightColor / 2;
    
    // ������Ӱ
    float shadow = ShadowCalculation(fs_in.FragPosLightSpace);
    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular))+0.4*color;
    FragColor = vec4(lighting, 1.0);
    // FragColor = vec4(ambient + diffuse,1);
}
