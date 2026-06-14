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

func differential(torque: float, brake_torque, wheels, diff: DiffParameters, delta: float):
	var diff_state = DIFF_STATE.LOCKED
	
	var tr1 = wheels[0].get_reaction_torque()
	var tr2 = wheels[1].get_reaction_torque()
	
	var delta_torque := 0.0
	var bias := 0.0
	
	if tr1 >= tr2:
		bias = tr1 / max(tr2,0.001)
	else:
		bias = tr2 / max(tr1,0.001)
	
	delta_torque = tr1 - tr2
	var t1 := torque * 0.5
	var t2 := torque * 0.5
	
	var ratio = diff.power_ratio
	if torque * sign(get_gearing()) < 0:
		ratio = diff.coast_ratio
	
	if diff.diff_type == DIFF_TYPE.OPEN_DIFF:
		diff_state = DIFF_STATE.OPEN
	elif diff.diff_type == DIFF_TYPE.LOCKED:
		diff_state = DIFF_STATE.LOCKED
	else: # Limited Slip Differential
		if abs(delta_torque) > diff.diff_preload and bias >= ratio:
			diff_state = DIFF_STATE.SLIPPING
	
	match diff_state:
		DIFF_STATE.OPEN:
			var w0 = wheels[0]
			var w1 = wheels[1]
			var spin0 = w0.get_spin()
			var spin1 = w1.get_spin()
			
			var spin_diff :float= spin0 - spin1
			var diff_stiffness := 0.1
			var base_t := torque * 0.5
			var correction := diff_stiffness * spin_diff * drive_inertia / delta
			correction = clampf(correction, -abs(torque) * 0.3, abs(torque) * 0.3)
			
			var tx0 :float= base_t - correction
			var tx1 :float= base_t + correction
			
			# 正确：只传4个参数
			w0.apply_torque(tx0, brake_torque * 0.5, drive_inertia, delta)
			w1.apply_torque(tx1, brake_torque * 0.5, drive_inertia, delta)
			
			var target_split := 0.5
			if abs(spin_diff) > 0.1:
				target_split = 0.5 - 0.1 * sign(spin_diff)
			_diff_split = lerp(_diff_split, target_split, 0.1)
		
		DIFF_STATE.SLIPPING:
			_diff_clutch.friction = diff.diff_preload
			var diff_torques = _diff_clutch.get_reaction_torques(wheels[0].get_spin(), wheels[1].get_spin(), tr1, tr2, diff.diff_preload * ratio, 0.0)
			t1 += diff_torques.x
			t2 += diff_torques.y
			# 正确：只传4个参数
			wheels[0].apply_torque(t1, brake_torque * 0.5, drive_inertia, delta)
			wheels[1].apply_torque(t2, brake_torque * 0.5, drive_inertia, delta)
		
		DIFF_STATE.LOCKED:
			var w0 = wheels[0]
			var w1 = wheels[1]
			var spin0 = w0.get_spin()
			var spin1 = w1.get_spin()
			var diff_speed = spin0 - spin1
			
			var axial_lock_stiffness := 200.0
			var correction = clamp(diff_speed * axial_lock_stiffness, -1500.0, 1500.0)
			
			var tx0 :float= torque * 0.5 - correction
			var tx1 :float= torque * 0.5 + correction
			
			# 正确：只传4个参数（没有多余的 true/false）
			w0.apply_torque(tx0, brake_torque * 0.5, drive_inertia, delta)
			w1.apply_torque(tx1, brake_torque * 0.5, drive_inertia, delta)

func drivetrain(torque: float, rear_brake_torque: float, front_brake_torque: float, wheels: Array, clutch_input: float, delta: float):
	var rear_wheels = [wheels[0], wheels[1]]
	var front_wheels = [wheels[2], wheels[3]]
	
	avg_rear_spin = (wheels[0].get_spin() + wheels[1].get_spin()) * 0.5
	avg_front_spin = (wheels[2].get_spin() + wheels[3].get_spin()) * 0.5 
	
	drive_inertia = (_engine_inertia + pow(abs(get_gearing()), 2) * drivetrain_params.gear_inertia) * (1 - clutch_input)
	var drive_torque = torque * get_gearing()
	
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
				
				var net_torque := 0.0
				var combined_wheel_inertias := 0.0
				var rolling_resistance := 0.0
				
				for w in wheels:
					net_torque += w.get_reaction_torque()
					combined_wheel_inertias += w.wheel_inertia
					rolling_resistance += w.rolling_resistance
					
				net_torque += drive_torque
				var brake_torque := rear_brake_torque + front_brake_torque
				var spin := 0.0
				
				if abs(avg_spin) < 5.0 and brake_torque > abs(net_torque):
					spin = 0.0
				else:
					net_torque -= (brake_torque + abs(rolling_resistance)) * sign(avg_spin)
					spin = avg_spin + delta * net_torque / (drive_inertia + combined_wheel_inertias)
				
				wheels[0].set_spin(spin)
				wheels[1].set_spin(spin)
				wheels[2].set_spin(spin)
				wheels[3].set_spin(spin)
			
			DIFF_TYPE.LIMITED_SLIP:
				var rear_drive = drive_torque * (1 - drivetrain_params.center_split_fr)
				var front_drive = drive_torque * drivetrain_params.center_split_fr
				
				differential(rear_drive, rear_brake_torque, rear_wheels, drivetrain_params.rear_diff, delta)
				differential(front_drive, front_brake_torque, front_wheels, drivetrain_params.front_diff, delta)
			
			DIFF_TYPE.OPEN_DIFF:
				var rear_drive = drive_torque * 0.5
				var front_drive = drive_torque * 0.5
				
				differential(rear_drive, rear_brake_torque, rear_wheels, drivetrain_params.rear_diff, delta)
				differential(front_drive, front_brake_torque, front_wheels, drivetrain_params.front_diff, delta)
