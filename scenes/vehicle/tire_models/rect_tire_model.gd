class_name RectTireModel
extends BaseTireModel

# ==================== 刷子模型参数 ====================

## 纵向刚度 [N per slip ratio]（单位滑移率对应的力）
@export var longitudinal_stiffness: float = 100000.0

## 侧偏刚度 [N per rad]（单位弧度对应的力）
@export var lateral_stiffness: float = 80000.0

## 基础摩擦系数（与路面摩擦、温度、磨损等相乘）
@export var friction_coefficient: float = 1.0

## 刚度随载荷变化的指数（0.7 表示重载时刚度增大）
@export var stiffness_load_exp: float = 0.7

## 最大气动拖距 [m]
@export var pneumatic_trail_max: float = 0.04

## 拖距随载荷变化的指数
@export var trail_load_exp: float = 0.5

# ==================== 主更新函数 ====================

func update_tire_forces(slip: Vector2, normal_load: float, surface_mu: float = 1.0) -> Vector3:
	# ----- 2. 滑移量提取（量纲同 TMeasy） -----
	var slip_angle := clampf(slip.x, -0.5, 0.5)    # 侧偏角 [rad]
	var slip_ratio := clampf(slip.y, -1.0, 1.0)   # 纵向滑移率 [-1,1]
	# ----- 1. 环境修正（与基类一致） -----
	var temp_mu := TIRE_TEMP_MU.sample_baked(tire_temp / max(max_tire_temp, 1.0))
	var wear_mu := TIRE_WEAR_CURVE.sample_baked(tire_wear)
	load_sensitivity = update_load_sensitivity(normal_load)   # 载荷对附着能力的修正
	var mu_total := surface_mu * load_sensitivity * wear_mu * temp_mu * friction_coefficient
	var max_force := mu_total * normal_load

	

	# ----- 3. 载荷比对刚度的影响 -----
	var load_ratio := clampf(normal_load / tire_rated_load, 0.3, 1.5)
	var stiff_factor := pow(load_ratio, stiffness_load_exp)
	var Cx_eff := longitudinal_stiffness * stiff_factor   # 有效纵向刚度
	var Cα_eff := lateral_stiffness * stiff_factor        # 有效侧偏刚度

	# ----- 4. 计算归一化滑移（摩擦椭圆） -----
	# 采用 tan(α) 作为侧向真实滑移，小角度下等于角度
	var sigma_x :float= Cx_eff * slip_ratio / max(max_force, 0.001)
	var sigma_y :float= Cα_eff * tan(slip_angle) / max(max_force, 0.001)
	var sigma := sqrt(sigma_x * sigma_x + sigma_y * sigma_y)

	# ----- 5. 刷子模型（均匀压力分布）联合滑移力 -----
	var fx := 0.0   # 纵向力
	var fy := 0.0   # 侧向力

	if sigma <= 1.0:
		# 完全粘着区（线性）
		fx = Cx_eff * slip_ratio
		fy = Cα_eff * tan(slip_angle)
	else:
		# 部分滑移区（饱和，公式确保力连续）
		var factor = (2.0 - 1.0 / sigma) / sigma
		fx = max_force * sigma_x * factor
		fy = max_force * sigma_y * factor

	# 数值保护
	fx = clampf(fx, -max_force * 1.1, max_force * 1.1)
	fy = clampf(fy, -max_force * 1.1, max_force * 1.1)

	# ----- 6. 回正力矩 Mz（与 TMeasy 风格一致） -----
	var Fy_abs_norm :float= abs(fy / max(max_force, 0.001))
	var trail := pneumatic_trail_max * pow(load_ratio, trail_load_exp)
	var trail_decay := exp(-4.0 * pow(Fy_abs_norm, 3.0))
	trail_decay = clampf(trail_decay, 0.05, 1.0)
	var aligning_moment := -fy * trail * trail_decay

	# 纵向力对回正力矩的小修正（驱动/制动影响）
	aligning_moment += fx * slip_angle * 0.02
	aligning_moment = clampf(aligning_moment, -max_force * 0.05, max_force * 0.05)

	# ----- 7. 返回 (Fy, Fx, Mz) 顺序与 TMeasy 一致 -----
	return Vector3(fy, fx, aligning_moment)
