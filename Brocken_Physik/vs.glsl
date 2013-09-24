#version 330 core

in vec3 positionMC;
in vec3 normalMC;
//in vec3 colorIn;
in vec2 texCoords;

out vec3 fragmentNormal;
//out vec3 fragmentColor;
out vec2 texCrd;

uniform mat4 model2world;
uniform mat4 viewProj;

void main(){
	gl_Position = viewProj * model2world * vec4(positionMC, 1);
	fragmentNormal = transpose(inverse(mat3(model2world))) * normalMC;
	//fragmentColor = colorIn;
	texCrd = texCoords;
}