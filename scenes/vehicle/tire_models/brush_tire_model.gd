class_name BrushTireModel
extends BaseTireModel

# ----- 原有参数（保持兼容）-----
@export_range(0.0, 1.0) var tire_stiffness := 0.4
@export var contact_patch := 0.22
@export var long_stiffness_factor := 13.0   # 量纲修复后需增大，见下方说明
@export var stiffness_load_exp := 0.7

# ----- 回正力矩参数 -----
@export var trail_base := 0.035
@export var trail_load_exp := 0.5
@export var base_stiffness := 2500000.0
# ----- 新增：稳定性与物理修正参数 -----
@export var brush_shape := 0.65   # 刷子模型形状因子，0.5~0.8 之间较稳定
func sign(val:float):
	if val >= 0:
		return 1;
	else:
		return -1;

func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:
	var load_ratio = clampf(normal_load / tire_rated_load, 0.3, 1.5)
	var stiff_factor = pow(load_ratio, stiffness_load_exp)
	
	# 侧偏刚度 C_α [N/rad]
	var lateral_stiffness = 0.5 * base_stiffness * pow(contact_patch, 2) * stiff_factor
	
	# 【量纲修复】纵向刚度 C_κ [N]，slip.y 无量纲
	var longitudinal_stiffness = lateral_stiffness * long_stiffness_factor / TAU
	
	# 线性力
	var Fx_lin = lateral_stiffness * slip.x        # [N]
	var Fy_lin = longitudinal_stiffness * slip.y   # [N]
	
	# 摩擦极限
	var wear_mu := TIRE_WEAR_CURVE.sample_baked(tire_wear)
	
	var temp_mu := TIRE_TEMP_MU.sample_baked(tire_temp / max(max_tire_temp,1.0))
	load_sensitivity = update_load_sensitivity(normal_load)
	var mu := surface_mu * load_sensitivity * wear_mu * temp_mu
	var max_friction_force := mu * normal_load
	
	# Brush Model 平滑饱和（替换硬摩擦圆截断）
	var linear_magnitude = sqrt(Fx_lin * Fx_lin + Fy_lin * Fy_lin)
	var Fx = Fx_lin
	var Fy = Fy_lin
	
	if linear_magnitude > 0.001 and max_friction_force > 0.001:
		
		var rho = linear_magnitude / max_friction_force
		
		var brush_factor: float
		if rho < 1.0:
			# 附着区：力略低于线性，平滑过渡
			brush_factor = 1.0 - (0.5 * rho * rho)
		else:
			# 滑移区：严格饱和
			
			brush_factor = 1.0 / max(rho,1.0)*sign(rho);
		
		Fx = Fx_lin * brush_factor
		Fy = Fy_lin * brush_factor
	
	# 回正力矩：大滑移角时拖距衰减（防止自转）
	var trail = trail_base * pow(load_ratio, trail_load_exp)
	var abs_sa = abs(slip.x)
	var trail_factor: float
	if abs_sa < 0.06:           # < 3.4°
		trail_factor = 1.0
	elif abs_sa < 0.35:         # < 20°
		var t = (abs_sa - 0.06) / 0.29
		trail_factor = 0.5 * (1.0 + cos(t * PI))
		trail_factor = max(trail_factor, 0.03)
	else:
		trail_factor = 0.03
	
	var Mz = -Fx * trail * trail_factor
	var max_mz = max_friction_force * trail_base * 1.5
	Mz = clampf(Mz, -max_mz, max_mz)
	
	return Vector3(Fx, Fy, Mz)
