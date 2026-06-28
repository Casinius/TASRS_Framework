class_name P2012TireModel
extends BaseTireModel
# 侧向（Lateral）参数
#@export var pacejka_b_lat := 10.0
#@export var pacejka_c_lat := 1.35
@export var pacejka_d_lat := 1.0
@export var pacejka_e_lat := 0.0

# 纵向（Longitudinal）参数
@export var pacejka_b_long := 7.0
@export var pacejka_c_long := 1.6
@export var pacejka_d_long := 1.0
@export var pacejka_e_long := 0.2

# 回正力矩参数（简化）
@export var aligning_moment_factor := 0.05   # 回正力矩系数


# --- 保留您现有的 Pacejka 参数 ---
@export var pacejka_b_lat := 8.0
@export var pacejka_c_lat := 1.35
# ... 其他现有参数

# --- 新增 MF6.1.2 需要的参数 (示例) ---
# 1. 轮胎名义载荷 (N)
@export var nominal_load := 4000.0 
# 2. 轮胎名义胎压 (bar)
@export var nominal_pressure := 2.2 
# 3. 载荷和压力对摩擦系数的影响系数 (示例值)
@export var load_sensitivity_coeff := 1.0 
@export var pressure_sensitivity_coeff := 1.0 

func pacejka_lat(slip_angle: float, normal_load: float, mu: float) -> float:
	var B = pacejka_b_lat
	var C = pacejka_c_lat
	var D = mu * normal_load   # 现在峰值直接由 mu 决定
	var E = pacejka_e_lat
	var arg = B * slip_angle
	return D * sin(C * atan(arg - E * (arg - atan(arg))))

func pacejka_long(slip_ratio: float, normal_load: float, mu: float) -> float:
	var B = pacejka_b_long
	var C = pacejka_c_long
	var D = mu * normal_load
	var E = pacejka_e_long
	var arg = B * slip_ratio
	return D * sin(C * atan(arg - E * (arg - atan(arg))))

func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0, tire_pressure: float = 2.2) -> Vector3:
	# 1. 计算最终的等效摩擦系数（包含所有修正）
	var load_factor = (normal_load / nominal_load) * load_sensitivity_coeff
	var pressure_factor = (tire_pressure / nominal_pressure) * pressure_sensitivity_coeff
	var temp_mu = TIRE_TEMP_MU.sample_baked(tire_temp / max_tire_temp)
	var wear_mu = TIRE_WEAR_CURVE.sample_baked(tire_wear)
	var mu_eff = surface_mu * load_factor * pressure_factor * wear_mu * temp_mu

	# 2. 计算纯滑移力（传入 mu_eff，内部 D = mu_eff * normal_load）
	var fy0 = pacejka_lat(slip.x, normal_load, mu_eff)
	var fx0 = pacejka_long(slip.y, normal_load, mu_eff)

	# 3. 组合滑移下的摩擦圆缩放（防止合力超过椭圆极限）
	var combined = sqrt(fx0*fx0 + fy0*fy0)
	var max_force = mu_eff * normal_load
	var fx = fx0
	var fy = fy0
	if combined > max_force and combined > 0.001:
		var scale = max_force / combined
		fx *= scale
		fy *= scale

	# 4. 回正力矩（可保留简化版，或扩展）
	var aligning_moment = -fy * slip.x * aligning_moment_factor
	aligning_moment = clamp(aligning_moment, -max_force * 0.05, max_force * 0.05)

	return Vector3(fy, fx, aligning_moment)
