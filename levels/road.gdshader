shader_type spatial;

uniform sampler2D albedo_texture : source_color;
uniform sampler2D normal_texture : hint_normal;
uniform float tile_size : hint_range(0.1, 5.0) = 0.5;
uniform float rotation_strength : hint_range(0.0, 1.0) = 1.0;
uniform float offset_strength : hint_range(0.0, 0.3) = 0.1;
uniform float seed_offset : hint_range(0.0, 100.0) = 0.0;

varying vec3 world_pos;

float random(vec2 st) {
    return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * 43758.5453123);
}

void vertex() {
    // 将顶点转换到世界空间
    world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
    // 使用世界坐标的 XZ 平面（如果是垂直表面可改为 XY 或 YZ）
    vec2 grid_coord = world_pos.xz / tile_size;
    vec2 grid_id = floor(grid_coord);
    vec2 local_uv = fract(grid_coord);   // 每个格子内的局部坐标 [0,1]

    // 随机值：基于格子ID
    float r1 = random(grid_id + seed_offset);
    float r2 = random(grid_id + seed_offset + 57.0);
    float r3 = random(grid_id + seed_offset + 111.0);

    // 随机旋转角度
    float angle = r1 * rotation_strength * 360.0;
    float rad = radians(angle);
    
    // 随机偏移（每个格子独立，范围 -offset_strength 到 +offset_strength）
    vec2 offset = (vec2(r2, r3) - 0.5) * offset_strength;

    // 旋转局部UV
    vec2 rotated_uv = vec2(
        local_uv.x * cos(rad) - local_uv.y * sin(rad),
        local_uv.x * sin(rad) + local_uv.y * cos(rad)
    );
    
    // 加上偏移，得到最终UV
    vec2 final_uv = rotated_uv + offset;

    // 采样纹理
    vec4 albedo_tex = texture(albedo_texture, final_uv);
    ALBEDO = albedo_tex.rgb;
    
    // 法线贴图同样使用相同的 final_uv
    vec3 normal_tex = texture(normal_texture, final_uv).rgb;
    NORMAL_MAP = normal_tex;
    
    // 可选：金属/粗糙度等，按需添加
}