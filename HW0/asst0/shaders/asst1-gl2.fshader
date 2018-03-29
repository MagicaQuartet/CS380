uniform float uColorTemp;
uniform float uTime;

varying vec3 vColor;

void main(void) {
  vec4 color = vec4(vColor.x, vColor.y, vColor.z, 1);
  if (uColorTemp == 1.0) {
    color = vec4(1, 0, 0, 1);
  }
  else if (uColorTemp == 2.0) {
    color = vec4(0, 1, 0, 1);
  }
  else if (uColorTemp == 3.0) {
    color = vec4(0, 0, 1, 1);
  }
  else if (uColorTemp == 4.0) {
    color = vec4(sin(uTime / 500.), cos(uTime / 500.), 1 - sin(uTime / 500.), 1);
  }
  gl_FragColor = color;
}
