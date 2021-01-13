// 12/31/20

#ifdef GL_ES
precision mediump float;
#endif

#define PI 3.14159265359

uniform vec2 u_resolution;
uniform float u_time;

vec2 rotate2D(vec2 _st, float _angle){
    _st -= 0.5;
    _st =  mat2(cos(_angle),-sin(_angle),
                sin(_angle),cos(_angle)) * _st;
    _st += 0.5;
    return _st;
}

float random (vec2 st) {
    return fract(sin(dot(st.xy,
                         vec2(12.9898,78.233)))*
        43758.5453123);
}

void main() {
    vec2 st = gl_FragCoord.xy/u_resolution.xy;
    
    vec3 color = vec3(0.87);
    
    st *= 1.0; // scale up the space by 3
    st = fract(st);
    // have 9 spaces go from 0-1
    
    for (int n = 1; n < 50; n ++){
        float i = float(n);
        st += vec2(0.7/i * sin(i*st.y + u_time + 0.5 * i), 0.8/i * sin(st.x + u_time + 0.2 * i) + 2.0 * 1.0);
       
    }
    st = rotate2D(st, PI * (cos(u_time)) * 0.825 / 100.0);
    st *= 40.0;
   
    color = vec3( 0.5 * sin(st.x + u_time) + st.x * 0.245, 0.5 * sin(st.y * 0.32478 + u_time) + 0.5, sin(st.x + st.y));

    gl_FragColor = vec4(color,1.0);
}
