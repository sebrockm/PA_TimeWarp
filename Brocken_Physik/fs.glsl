#version 330 core

const vec3 inverseLightDir = normalize(vec3(-1, 1, -1));

uniform sampler2D materialTex;
in vec2 texCrd;

in vec3 fragmentNormal;
//in vec3 fragmentColor;

out vec4 color;

void main(){
	vec3 norm = normalize(fragmentNormal);
	float intens = max(dot(inverseLightDir, norm), .3);
	//color = vec4(intens*(fragmentColor+(norm+vec3(1))/2)/2, 1);
	color = texture(materialTex, texCrd)*intens;
}