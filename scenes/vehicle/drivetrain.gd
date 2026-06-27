class_name DriveTrain
extends Node

const AV_2_RPM: float = 60 / TAU

enum DIFF_TYPE{
	LIMITED_SLIP,
	OPEN_DIFF,
	LOCKED,
}

enum DIFF_STATE {
	LOCKED,
	SLIPPING,
	OPEN,
}

enum DRIVE_TYPE{
	FWD,
	RWD,
	AWD,
}

@export var drivetrain_params: DriveTrainParameters

var selected_gear := 0
var _diff_clutch := Clutch.new() 
var _engine_inertia := 0.20
var _diff_split := 0.5
var last_shift_time := 0

var avg_rear_spin := 0.0
var avg_front_spin := 0.0

var drive_inertia := 10.0
var reaction_torque := 0.0

func automatic_shifting(cur_torque, lower_gear_torque, higher_gear_torque, rpm, max_rpm, brake_input, speed):
	if !drivetrain_params.automatic:
		return
		
	var reversing = false
	var shift_time = 700
	
	if selected_gear == -1:
		reversing = true

	if higher_gear_torque > cur_torque and selected_gear >= 0:
		if rpm > 0.85 * max_rpm:
			if Time.get_ticks_msec() - last_shift_time > shift_time:
				shift_up()
	
	if selected_gear > 1 and rpm < 0.5 * max_rpm and lower_gear_torque > cur_torque:
		if Time.get_ticks_msec() - last_shift_time > shift_time:
			shift_down()
	
	if abs(selected_gear) <= 1 and abs(speed) < 3.0 and brake_input > 0.2:
		if not reversing:
			if Time.get_ticks_msec() - last_shift_time > shift_time:
				shift_down()
		else:
			if Time.get_ticks_msec() - last_shift_time > shift_time:
				shift_up()


func set_selected_gear(gear):
	gear = clamp(gear, -1, drivetrain_params.gear_ratios.size())
	selected_gear = gear


func shift_up():
	if selected_gear < drivetrain_params.gear_ratios.size():
		selected_gear += 1
		last_shift_time = Time.get_ticks_msec()
		set_selected_gear(selected_gear)


func shift_down():
	if selected_gear > -1:
		selected_gear -= 1
		last_shift_time = Time.get_ticks_msec()
		set_selected_gear(selected_gear)


func get_gearing() -> float:
	if selected_gear > drivetrain_params.gear_ratios.size():
		return 0.0
	if selected_gear > 0:
		return drivetrain_params.gear_ratios[selected_gear - 1] * drivetrain_params.final_drive
	if selected_gear == -1:
		return -drivetrain_params.reverse_ratio * drivetrain_params.final_drive
	return 0.0


func set_input_inertia(value):
	_engine_inertia =  value

func sign_of(value:float):
	if value >= 0:
		return 1;
	else:
		return -1;

# 在 DriveTrain 类中，替换原有 differential 方法
func differential(torque: float, brake_torque: float, wheels: Array, diff: DiffParameters, delta: float):
	var w_left  = wheels[0]
	var w_right = wheels[1]
	var spin_L  = w_left.get_spin()
	var spin_R  = w_right.get_spin()
	var spin_diff = spin_L - spin_R
	var abs_spin_diff = abs(spin_diff)

	# 计算左右轮反应扭矩（地面阻力）
	var react_L = w_left.get_reaction_torque()
	var react_R = w_right.get_reaction_torque()

	# 基础分配（开式差速器：各一半）
	var base_T = torque * 0.5

	# 根据差速器类型决定分配方式
	match diff.diff_type:
		DIFF_TYPE.OPEN_DIFF:
			# 开式：平均分配，但允许内部摩擦（仅作为阻尼）
			# 实际开式差速器允许轮速自由差异，不产生偏置。
			# 为了数值稳定，加一点阻尼防止高频震荡。
			var damping = 0.1 * (spin_L - spin_R) * drive_inertia / delta
			damping = clampf(damping, -abs(torque)*0.1, abs(torque)*0.1)
			var T_L = base_T - damping
			var T_R = base_T + damping
			w_left.apply_torque(T_L, brake_torque * 0.5, drive_inertia, delta)
			w_right.apply_torque(T_R, brake_torque * 0.5, drive_inertia, delta)

		DIFF_TYPE.LOCKED:
			# 锁止：强制同速，用大刚度修正
			var lock_stiffness = 2000.0
			var correction = spin_diff * lock_stiffness * delta
			correction = clampf(correction, -abs(torque)*0.5, abs(torque)*0.5)
			var T_L = base_T - correction
			var T_R = base_T + correction
			w_left.apply_torque(T_L, brake_torque * 0.5, drive_inertia, delta)
			w_right.apply_torque(T_R, brake_torque * 0.5, drive_inertia, delta)

		DIFF_TYPE.LIMITED_SLIP:
			# ---- 基于扭矩偏置比的 LSD ----
			var max_TBR = diff.power_ratio   # 最大扭矩偏置比（例如 2.5）
			# 负载转移：根据左右轮反应扭矩差决定偏置方向
			var react_ratio = 1.0
			if abs(react_L) > 0.01 and abs(react_R) > 0.01:
				# 高附着力侧会提供更大的反应扭矩
				var high_react = max(react_L, react_R)
				var low_react = min(react_L, react_R)
				react_ratio = high_react / max(low_react, 0.01)
				react_ratio = clamp(react_ratio, 1.0, max_TBR)

			# 转速差影响：快速打滑时增大偏置
			var speed_factor = 1.0
			if abs_spin_diff > 0.5:   # 转速差超过阈值
				speed_factor = 1.0 + 0.5 * abs_spin_diff   # 线性增加
				speed_factor = clamp(speed_factor, 1.0, max_TBR)

			# 综合偏置比（取两者中较大者，但受上限）
			var TBR = max(react_ratio, speed_factor)
			TBR = clamp(TBR, 1.0, max_TBR)

			# 确定哪一侧需要更多扭矩（低转速侧得更多）
			var T_high
			var T_low
			if spin_L > spin_R:
				# 右轮转速慢，应得到更多扭矩（右轮高）
				T_high = base_T * TBR / (1.0 + TBR) * 2.0   # 使总和等于 torque
				T_low = torque - T_high
				# 但必须保证 T_high + T_low = torque
				# 由于浮点误差，重新正规化
				var total = T_high + T_low
				if total != 0:
					T_high = T_high / total * torque
					T_low = T_low / total * torque
				# 分配：右轮得高扭矩，左轮得低
				w_left.apply_torque(T_low, brake_torque * 0.5, drive_inertia, delta)
				w_right.apply_torque(T_high, brake_torque * 0.5, drive_inertia, delta)
			else:
				# 左轮转速慢，左轮得高扭矩
				T_high = base_T * TBR / (1.0 + TBR) * 2.0
				T_low = torque - T_high
				var total = T_high + T_low
				if total != 0:
					T_high = T_high / total * torque
					T_low = T_low / total * torque
				w_left.apply_torque(T_high, brake_torque * 0.5, drive_inertia, delta)
				w_right.apply_torque(T_low, brake_torque * 0.5, drive_inertia, delta)

func drivetrain(torque: float, rear_brake_torque: float, front_brake_torque: float, wheels: Array, clutch_input: float, delta: float):
	var rear_wheels = [wheels[0], wheels[1]]
	var front_wheels = [wheels[2], wheels[3]]
	
	avg_rear_spin = (wheels[0].get_spin() + wheels[1].get_spin()) * 0.5
	avg_front_spin = (wheels[2].get_spin() + wheels[3].get_spin()) * 0.5 
	
	drive_inertia = (_engine_inertia + pow(abs(get_gearing()), 2) * drivetrain_params.gear_inertia) * (1 - clutch_input)
	
	
	
	var drive_torque := torque * get_gearing()
	
	if drivetrain_params.drivetype == DRIVE_TYPE.RWD:
		differential(drive_torque, rear_brake_torque, rear_wheels, drivetrain_params.rear_diff, delta)
		front_wheels[0].apply_torque(0.0, front_brake_torque * 0.5, 0.0, delta)
		front_wheels[1].apply_torque(0.0, front_brake_torque * 0.5, 0.0, delta)
		reaction_torque = (rear_wheels[0].get_reaction_torque() + rear_wheels[1].get_reaction_torque()) * 0.5
		reaction_torque *= (1.0 / get_gearing())
	
	elif drivetrain_params.drivetype == DRIVE_TYPE.FWD:
		differential(drive_torque, front_brake_torque, front_wheels, drivetrain_params.front_diff, delta)
		rear_wheels[0].apply_torque(0.0, rear_brake_torque * 0.5, 0.0, delta)
		rear_wheels[1].apply_torque(0.0, rear_brake_torque * 0.5, 0.0, delta)
		reaction_torque = (front_wheels[0].get_reaction_torque() + front_wheels[1].get_reaction_torque()) * 0.5
		reaction_torque *= (1.0 / get_gearing())
		
	elif drivetrain_params.drivetype == DRIVE_TYPE.AWD:
		reaction_torque = (rear_wheels[0].get_reaction_torque() + rear_wheels[1].get_reaction_torque()) * 0.25
		reaction_torque += (front_wheels[0].get_reaction_torque() + front_wheels[1].get_reaction_torque()) * 0.25
		reaction_torque *= (1.0 / get_gearing())
		
		match drivetrain_params.center_diff.diff_type:
			DIFF_TYPE.LOCKED: # Locked center diff currently means raw 4x4 
				var avg_spin = (avg_front_spin + avg_rear_spin) * 0.5
				var lock_stiffness = 2000.0
				var torque_per_wheel = drive_torque / 4.0
				for w in wheels:
					var spin_diff = w.get_spin() - avg_spin
					var lock_torque = spin_diff * lock_stiffness * delta
					lock_torque = clampf(lock_torque, -torque_per_wheel, torque_per_wheel)
					var T = torque_per_wheel - lock_torque
					# 此处 brake_torque 需根据前后轴分开传，简单起见可用总制动扭矩/4
					w.apply_torque(T, 0.0, drive_inertia/4, delta)
			
			DIFF_TYPE.LIMITED_SLIP:
				var rear_drive := drive_torque * (1 - drivetrain_params.center_split_fr)
				var front_drive := drive_torque * drivetrain_params.center_split_fr
				
				differential(rear_drive, rear_brake_torque, rear_wheels, drivetrain_params.rear_diff, delta)
				differential(front_drive, front_brake_torque, front_wheels, drivetrain_params.front_diff, delta)
			
			DIFF_TYPE.OPEN_DIFF:
				var rear_drive := drive_torque * 0.5
				var front_drive := drive_torque * 0.5
				
				differential(rear_drive, rear_brake_torque, rear_wheels, drivetrain_params.rear_diff, delta)
				differential(front_drive, front_brake_torque, front_wheels, drivetrain_params.front_diff, delta)
