#version 130

uniform float uVertexScale;
uniform float uFragScale;

in vec2 aPosition;
in vec3 aColor;
in vec2 aTexCoord0, aTexCoord1;

out vec3 vColor;
out vec2 vTexCoord0, vTexCoord1;

void main() {
  if (uVertexScale > 1) {
	gl_Position = vec4(aPosition.x, aPosition.y * 1 / uVertexScale, 0, 1);
  }
  else {
	gl_Position = vec4(aPosition.x * uVertexScale, aPosition.y, 0, 1);
  }
  vColor = aColor;
  vTexCoord0 = aTexCoord0;
  vTexCoord1 = aTexCoord1;
}
