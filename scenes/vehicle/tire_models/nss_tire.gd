class_name NSSTireModel
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

# ---- 新增低速稳定参数 ----
@export var low_speed_threshold := 0.02      # 滑移量阈值（弧度或比率），低于此值使用线性刚度
@export var linear_stiffness_lat := 120.0    # 侧偏刚度（N/rad），需根据轮胎特性调校
@export var linear_stiffness_long := 80.0    # 纵向刚度（N/单位滑移率）

# 纯侧偏力公式（Pacejka 89/96），带低速线性过渡
func pacejka_lat(slip_angle: float, normal_load: float) -> float:
	var B = pacejka_b_lat
	var C = pacejka_c_lat
	var D = pacejka_d_lat * normal_load
	var E = pacejka_e_lat
	var arg = B * slip_angle
	var f_pacejka = D * sin(C * atan(arg - E * (arg - atan(arg))))
	
	# 低速线性近似：使用侧偏刚度 linear_stiffness_lat * slip_angle
	var f_linear = linear_stiffness_lat * slip_angle
	# 但线性刚度应受载荷影响，简单线性缩放
	var load_scale = normal_load / 4000.0  # 假设额定载荷4000N，可调
	f_linear *= max(0.1, load_scale)      # 避免载荷为零
	
	# 平滑过渡（smoothstep）
	var t = clamp(abs(slip_angle) / low_speed_threshold, 0.0, 1.0)
	t = t * t * (3.0 - 2.0 * t)  # smoothstep
	# 当 |slip| < threshold 时 t→0，使用线性；当 > threshold 时 t→1，使用 Pacejka
	var f = lerp(f_linear, f_pacejka, t)
	return f

# 纯纵滑力公式（Pacejka），带低速线性过渡
func pacejka_long(slip_ratio: float, normal_load: float) -> float:
	var B = pacejka_b_long
	var C = pacejka_c_long
	var D = pacejka_d_long * normal_load
	var E = pacejka_e_long
	var arg = B * slip_ratio
	var f_pacejka = D * sin(C * atan(arg - E * (arg - atan(arg))))
	
	# 线性近似
	var f_linear = linear_stiffness_long * slip_ratio
	var load_scale = normal_load / 4000.0
	f_linear *= max(0.1, load_scale)
	
	var t = clamp(abs(slip_ratio) / low_speed_threshold, 0.0, 1.0)
	t = t * t * (3.0 - 2.0 * t)
	var f = lerp(f_linear, f_pacejka, t)
	return f

func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:
	
	# 1. 温度、磨损、载荷灵敏度修正
	var temp_mu := TIRE_TEMP_MU.sample_baked(tire_temp / max_tire_temp)
	var wear_mu := TIRE_WEAR_CURVE.sample_baked(tire_wear)
	load_sensitivity = update_load_sensitivity(normal_load)
	var mu = surface_mu * load_sensitivity * wear_mu * temp_mu

	# 2. 输入滑移：slip.x = 侧偏角（弧度），slip.y = 纵向滑移率
	var slip_angle = slip.x
	var slip_ratio = slip.y

	# 3. 计算纯侧偏力和纯纵滑力（带低速稳定处理）
	var fy0 = pacejka_lat(slip_angle, normal_load)
	var fx0 = pacejka_long(slip_ratio, normal_load)

	# 4. 组合滑移下的椭圆摩擦圆限制（更平滑）
	var max_force = mu * normal_load
	var fx = fx0
	var fy = fy0
	var combined_force_sq = fx0*fx0 + fy0*fy0
	if combined_force_sq > max_force * max_force and combined_force_sq > 1e-6:
		# 采用椭圆投影：保持方向，将模长缩放到 max_force，同时考虑椭圆形状（可自定义）
		# 此处直接缩放为圆，但也可以根据侧偏/纵滑刚度差异做椭圆，这里简化为圆
		var scale = max_force / sqrt(combined_force_sq)
		fx *= scale
		fy *= scale
	# 小力时不做处理，避免除零

	# 5. 回正力矩（改进：基于侧偏力与滑移角，并增加低速衰减）
	# 使用侧偏刚度计算自回正臂，避免纯经验式
	var aligning_moment = 0.0
	if abs(slip_angle) > 1e-6:
		# 简单模型：力矩 = -侧向力 * 气胎拖距（近似为侧偏刚度/侧向力比例）
		# 此处采用原因子，但增加低速平滑因子，防止零速震荡
		var factor = aligning_moment_factor
		# 当滑移角很小时，力矩应趋于0，且平滑
		var speed_factor = clamp(abs(slip_angle) / low_speed_threshold, 0.0, 1.0)
		speed_factor = speed_factor * speed_factor * (3.0 - 2.0 * speed_factor)
		aligning_moment = -fy * slip_angle * factor * speed_factor
		# 限制最大力矩
		aligning_moment = clamp(aligning_moment, -max_force * 0.05, max_force * 0.05)
	# 否则力矩为0

	# 6. 返回 Vector3(x=侧向力, y=纵向力, z=回正力矩)
	return Vector3(fy, fx, aligning_moment)
