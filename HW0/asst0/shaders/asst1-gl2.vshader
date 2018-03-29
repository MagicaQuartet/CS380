uniform float uVertexTemp;
uniform float uTime;

attribute vec2 aPosition;
attribute vec3 aColor;

varying vec3 vColor;

void main() {
  gl_Position = vec4(aPosition.x + uVertexTemp, aPosition.y, 0, 1);
  vColor = aColor;
}
