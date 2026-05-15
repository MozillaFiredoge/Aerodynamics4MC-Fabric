#version 330

#moj_import <minecraft:dynamictransforms.glsl>

in vec4 vertexColor;
in vec2 texCoord0;

out vec4 fragColor;

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453123);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
}

void main() {
    float v = texCoord0.y * 2.0 - 1.0;
    float crossSection = pow(max(0.0, 1.0 - abs(v)), 0.95);
    float sweep = fract(texCoord0.x);
    float body = smoothstep(0.08, 0.28, sweep) * (1.0 - smoothstep(0.82, 0.98, sweep));
    float head = smoothstep(0.76, 0.98, sweep);
    float streak = max(body * 0.75, head);
    float n = noise(vec2(texCoord0.x * 3.0, texCoord0.y * 8.0));
    float shimmer = 0.84 + 0.30 * n;
    float alpha = min(vertexColor.a * crossSection * streak * shimmer * 2.65, 1.0);
    if (alpha <= 0.001) {
        discard;
    }
    vec3 headColor = mix(vertexColor.rgb, vec3(1.0, 0.95, 0.82), 0.45);
    vec3 color = mix(vertexColor.rgb, headColor, head) * (1.10 + 0.18 * n);
    fragColor = vec4(color, alpha) * ColorModulator;
}
