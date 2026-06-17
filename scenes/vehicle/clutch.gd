class_name Clutch
extends Node

@export var friction := 250.0

var locked := true
var prev_av := 0.0

func get_reaction_torques(av1: float, av2: float, t1: float, t2: float, slip_torque: float, kick := 0.0):
	var clutch_torque := friction + kick
	var delta_torque := t1 - t2
	var delta_av := av1 - av2
	var reaction_torques := Vector2.ZERO
	# 如果车轮转速几乎为零且扭矩过大，强制离合器打滑
	if abs(av1) < 0.5 and abs(av2) < 0.5 and abs(t1 - t2) > slip_torque:
		locked = false
	# Locked situations are handled in car and drivetrain scripts atm
	if locked:
		if absf(delta_torque) >= slip_torque:
			locked = false
	else:
		if absf(delta_av) < 0.5:
			locked = true
	
	if av1 < av2:
		reaction_torques.x = -clutch_torque
		reaction_torques.y = clutch_torque
	else:
		reaction_torques.x = clutch_torque
		reaction_torques.y = -clutch_torque
	return reaction_torques
	
	
	
func get_clutch_torque(engine_av: float, gearbox_av: float,engine_torque: float, clutch_pedal: float) -> float:
		# 踏板影响最大可传递扭矩
	var max_friction_torque := friction;
	var effective_friction :float= max_friction_torque * max(0.0, 1.0 - clutch_pedal)
	var speed_error := engine_av - gearbox_av
		
	if locked:
			# 打破锁定：扭矩超限 或 转速差过大
		if abs(engine_torque) > effective_friction or abs(speed_error) > 2.0:
			locked = false
		else:
				# 刚性连接：传递全部引擎扭矩
			return engine_torque
		
		# 打滑状态：库仑摩擦，扭矩大小恒定，方向与转速差相反
	var slip_torque_r :float= sign(speed_error) * effective_friction
		
		# 重新锁定条件：转速差小 + 扭矩在极限内
	if abs(speed_error) < 0.3 and abs(engine_torque) < effective_friction * 0.95:
		locked = true
		
	return slip_torque_r
