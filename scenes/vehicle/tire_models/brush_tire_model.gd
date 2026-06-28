class_name TMeasyTireModel
extends BaseTireModel

# ==================== TMeasy 参数 ====================

# ----- 侧向（Lateral）特性 -----
@export var Fy_peak_slip := 0.09       # 侧向峰值侧偏角 [rad] (典型 0.08~0.12)
@export var Fy_shape := 2.2           # 侧向曲线形状因子（越大越尖锐）

# ----- 纵向（Longitudinal）特性 -----
@export var Fx_peak_slip := 0.2      # 纵向峰值滑移率 (典型 0.08~0.15)
@export var Fx_shape := 1.5            # 纵向曲线形状因子

# ----- 载荷敏感性 -----
@export var peak_slip_load_exp := -0.2  # 峰值滑移率随载荷变化指数（负值：重载峰值降低）
@export var stiffness_load_exp := 0.7   # 刚度随载荷变化指数

# ----- 回正力矩参数 -----
@export var pneumatic_trail_max := 0.04 # 最大气动力拖距 [m]
@export var trail_load_exp := 0.5       # 拖距随载荷变化指数
#@export var Mz_saturation_slip := 0.15  # 回正力矩开始饱和的侧偏角 [rad]

# ----- 联合滑移参数 -----
#@export var combined_slip_shape := 1.0  # 联合滑移曲线形状（1.0 = 标准椭圆）
@export var force_curve_shape_div_factor := 9.2;
# ----- 兼容性参数（从你的 Pacejka 模型继承）-----
#@export var base_stiffness := 2500000.0
#@export var contact_patch := 0.22
#@export var long_stiffness_factor := 13.0

# ==================== 核心曲线函数 ====================

# TMeasy S 曲线：归一化力 = f(归一化滑移)
# slip_norm: 滑移率 / 峰值滑移率
# shape: 曲线锐度（越大峰值越尖锐）
func tm_easy_curve(slip_norm: float, shape: float) -> float:
	var abs_s = abs(slip_norm)
	if abs_s < 0.001:
		return slip_norm
	
	var k = max(0.0, 0.3 * (shape - 1.0))
	var force_norm = tanh(abs_s * shape) / (1.0 + k * abs_s)
	
	# 计算 f(1) 并归一化，确保 slip_norm=1 时输出 1
	var f1 = tanh(shape) / (1.0 + k)
	if f1 < 0.001:
		f1 = 0.001
	force_norm = force_norm / f1
	return force_norm * sign(slip_norm)
# 峰值滑移率随载荷变化（重载时峰值滑移率降低，更真实）
func get_peak_slip(load_ratio: float, base_peak: float, exp: float) -> float:
	return base_peak * pow(load_ratio, exp)

# ==================== 主更新函数 ====================

func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:

	var fixed_slip := Vector2(clampf(slip.x, -0.5, 0.5), clampf(slip.y, -1.0, 1.0))  # 仅翻转纵向
	#var fixed_slip := Vector2(slip.x, clampf(slip.y, -1.0, 1.0))
	# ----- 1. 环境修正（与你的 Pacejka 完全一致）-----
	var temp_mu := TIRE_TEMP_MU.sample_baked(tire_temp / max(max_tire_temp, 1.0))
	var wear_mu := TIRE_WEAR_CURVE.sample_baked(tire_wear)
	load_sensitivity = update_load_sensitivity(normal_load)
	var mu := surface_mu * load_sensitivity * wear_mu * temp_mu
	var max_force := mu * normal_load

	# ----- 2. 输入解析（与你的 Pacejka 完全一致）-----
	var slip_angle := fixed_slip.x    # 侧偏角 [rad]
	var slip_ratio := fixed_slip.y    # 纵向滑移率 [0~1]

	# ----- 3. 载荷比 -----
	var load_ratio := clampf(normal_load / tire_rated_load, 0.3, 1.5)
	var stiff_factor := pow(load_ratio, stiffness_load_exp)

	# ----- 4. 峰值滑移率随载荷变化 -----
	var sy_peak := get_peak_slip(load_ratio, Fy_peak_slip, peak_slip_load_exp)
	var sx_peak := get_peak_slip(load_ratio, Fx_peak_slip, peak_slip_load_exp)

	# ----- 5. 载荷比与峰值滑移率（保留，影响曲线形状）-----
	#var load_ratio := clampf(normal_load / tire_rated_load, 0.3, 1.5)
	#var sy_peak := get_peak_slip(load_ratio, Fy_peak_slip, peak_slip_load_exp)
	#var sx_peak := get_peak_slip(load_ratio, Fx_peak_slip, peak_slip_load_exp)

	# ----- 6. 归一化滑移率（方向分量）-----
	var sy_norm :float= slip_angle / max(sy_peak, 0.001)
	var sx_norm :float= slip_ratio / max(sx_peak, 0.001)

	# ----- 7. TMeasy 联合滑移（能量约束）-----
	var rho := sqrt(sx_norm * sx_norm + sy_norm * sy_norm)
	#var total_force_norm := tm_easy_curve(rho, (Fy_shape + Fx_shape) / 2.0)
	var total_force_norm := tm_easy_curve(rho, (Fy_shape + Fx_shape) / force_curve_shape_div_factor)
	# 方向分解（单位向量）
	var fx_dir := 0.0
	var fy_dir := 0.0
	if rho > 0.001:
		fx_dir = sx_norm / rho
		fy_dir = sy_norm / rho

	# ----- 8. 总力受限于最大附着力（能量约束）-----
	var total_force = total_force_norm * max_force
	var fx = total_force * fx_dir
	var fy = total_force * fy_dir

	# ----- 11. 回正力矩 Mz（TMeasy 风格，与你的 Pacejka 兼容）-----
	# TMeasy 核心：Mz = -Fy × pneumatic_trail
	# 拖距随侧向力饱和而衰减（非随侧偏角衰减，这是关键区别）
	#var trail := pneumatic_trail_max * pow(load_ratio, trail_load_exp)
	
	# 拖距衰减因子：基于侧向力的归一化程度（而非侧偏角）
	var Fy_abs_norm :float= abs(fy / max_force)
	var trail := pneumatic_trail_max * pow(load_ratio, trail_load_exp)
	var trail_decay := exp(-4.0 * pow(Fy_abs_norm, 3.0))
	trail_decay = clamp(trail_decay, 0.05, 1.0)
	var aligning_moment :float= -fy * trail * trail_decay
	
	# 附加：纵向滑移对回正力矩的影响（驱动/制动时回正力矩变化）
	# 小量修正，增强物理真实感
	# 纵向影响保留，限制不变
	aligning_moment += fx * slip_angle * 0.02
	aligning_moment = clampf(aligning_moment, -max_force * 0.05, max_force * 0.05)
	# 6. 返回 Vector3(x=侧向力, y=纵向力, z=回正力矩)
	#    注意：原代码中 force_vec.x 是侧向力（用于转向），force_vec.y 是纵向力（驱动/制动）
	# ----- 12. 返回（与你的 Pacejka 完全一致：x=Fy, y=Fx, z=Mz）-----
	return Vector3(fy, fx, aligning_moment)
