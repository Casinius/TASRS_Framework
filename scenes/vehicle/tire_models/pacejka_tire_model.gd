class_name PacejkaTireModel
extends BaseTireModel

# 侧向（Lateral）参数
@export var pacejka_b_lat := 10.0
@export var pacejka_c_lat := 1.35
@export var pacejka_d_lat := 1.0
@export var pacejka_e_lat := 0.0

# 纵向（Longitudinal）参数
@export var pacejka_b_long := 10.0
@export var pacejka_c_long := 1.65
@export var pacejka_d_long := 1.0
@export var pacejka_e_long := 0.0

# 回正力矩参数（简化）
@export var aligning_moment_factor := 0.05   # 回正力矩系数


# 纯侧偏力公式（Pacejka 89/96）
func pacejka_lat(slip_angle: float, normal_load: float) -> float:
	var B = pacejka_b_lat
	var C = pacejka_c_lat
	var D = pacejka_d_lat * normal_load
	var E = pacejka_e_lat
	var arg = B * slip_angle
	return D * sin(C * atan(arg - E * (arg - atan(arg))))


# 纯纵滑力公式
func pacejka_long(slip_ratio: float, normal_load: float) -> float:
	var B = pacejka_b_long
	var C = pacejka_c_long
	var D = pacejka_d_long * normal_load
	var E = pacejka_e_long
	var arg = B * slip_ratio
	return D * sin(C * atan(arg - E * (arg - atan(arg))))


func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:
	# 1. 温度、磨损、载荷灵敏度修正
	var temp_mu := TIRE_TEMP_MU.sample_baked(tire_temp / max_tire_temp)
	var wear_mu := TIRE_WEAR_CURVE.sample_baked(tire_wear)
	load_sensitivity = update_load_sensitivity(normal_load)
	var mu = surface_mu * load_sensitivity * wear_mu * temp_mu

	# 2. 输入滑移：slip.x = 侧偏角（弧度），slip.y = 纵向滑移率
	var slip_angle = slip.x
	var slip_ratio = slip.y

	# 3. 计算纯侧偏力和纯纵滑力（未修正组合滑移）
	var fy0 = pacejka_lat(slip_angle, normal_load)
	var fx0 = pacejka_long(slip_ratio, normal_load)

	# 4. 组合滑移下的摩擦圆缩放（最稳健、不易震荡）
	var combined_force = sqrt(fx0 * fx0 + fy0 * fy0)
	var max_force = mu * normal_load
	
	var fx = fx0
	var fy = fy0
	if combined_force > max_force and combined_force > 0.001:
		var scale = max_force / combined_force
		fx *= scale
		fy *= scale

	# 5. 回正力矩（简化模型，正比于侧向力，并随滑移角饱和）
	var aligning_moment = -fy * slip_angle * aligning_moment_factor
	# 限制最大回正力矩，防止失控
	aligning_moment = clamp(aligning_moment, -max_force * 0.05, max_force * 0.05)

	# 6. 返回 Vector3(x=侧向力, y=纵向力, z=回正力矩)
	#    注意：原代码中 force_vec.x 是侧向力（用于转向），force_vec.y 是纵向力（驱动/制动）
	return Vector3(fy, fx, aligning_moment)
