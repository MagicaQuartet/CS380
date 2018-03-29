#version 130

uniform float uVertexTemp;
uniform float uTime;

in vec2 aPosition;
in vec3 aColor;

out vec3 vColor;

void main() {
  gl_Position = vec4(aPosition.x, aPosition.y, 0,1);
  vColor = aColor; 
}
