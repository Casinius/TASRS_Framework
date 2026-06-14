class_name BrushTireModel
extends BaseTireModel

@export_range(0.0, 1.0) var tire_stiffness := 0.2
@export var contact_patch := 0.2

# 可选：纵向刚度系数（通常略大于侧向）
@export var long_stiffness_factor := 1.0


func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:
	# ----- 1. 基础刚度（沿用原参数风格）-----
	var base_stiffness = 1000000 + 8000000 * tire_stiffness
	var lateral_stiffness = 0.5 * base_stiffness * pow(contact_patch, 2)   # N/rad
	var longitudinal_stiffness = lateral_stiffness * long_stiffness_factor   # N / (滑移率)
	
	# ----- 2. 综合摩擦系数（温度/磨损/载荷/路面）-----
	var wear_mu = TIRE_WEAR_CURVE.sample_baked(tire_wear)
	var temp_mu = TIRE_TEMP_MU.sample_baked(tire_temp / max_tire_temp)
	load_sensitivity = update_load_sensitivity(normal_load)
	var mu = surface_mu * load_sensitivity * wear_mu * temp_mu
	var max_friction_force = mu * normal_load
	
	# ----- 3. 线性期望力（基于当前滑移）-----
	var Fx_lin = lateral_stiffness * slip.x        # 侧向力 (N), 与 slip.x 同号
	var Fy_lin = longitudinal_stiffness * slip.y   # 纵向力 (N), 与 slip.y 同号
	
	# ----- 4. 摩擦圆限制 -----
	var linear_magnitude = sqrt(Fx_lin * Fx_lin + Fy_lin * Fy_lin)
	var scale = 1.0
	if linear_magnitude > max_friction_force and linear_magnitude > 0.001:
		scale = max_friction_force / linear_magnitude
	
	var Fx = Fx_lin * scale
	var Fy = Fy_lin * scale
	
	# ----- 5. 回正力矩（简化但稳定）-----
	# 拖距随载荷增大而减小，随侧偏角增大而减小，这里取固定典型值
	#var pneumatic_trail = 0.035   # 米
	#var Mz = -Fx * pneumatic_trail
	var Mz :float= -Fx;
	# 可选：当侧偏角很大时拖距趋近0，这里不做复杂处理以保证稳定
	var slip_angle = abs(slip.x)
	var max_slip_for_zero_trail = 0.2
	var trail_factor := 1.0 - clampf(slip_angle / max_slip_for_zero_trail, 0.0, 1.0)
	trail_factor = pow(trail_factor, 1.5)   # 让衰减更柔和
	var base_pneumatic_trail := 0.035
	Mz = -Fx * base_pneumatic_trail * trail_factor
	
	return Vector3(Fx, Fy, Mz)
