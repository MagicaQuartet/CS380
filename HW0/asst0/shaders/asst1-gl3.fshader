#version 130

uniform float uColorTemp;
uniform float uTime;

in vec3 vColor;

out vec4 fragColor;

void main(void) {
  vec4 color = vec4(vColor.x, vColor.y, vColor.z, 1);
  fragColor = color;
}
