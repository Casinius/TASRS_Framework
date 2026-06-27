class_name Clutch
extends Node

@export var friction :float= 250.0          # 最大静/动摩擦扭矩 (N·m)
@export var dv_threshold := 5.0        # 速度差阈值 (rad/s)，小于此值视为黏滞

func get_clutch_torque(engine_av: float, gearbox_av: float, engine_torque: float, clutch_pedal: float) -> Array:
	# 有效摩擦扭矩（受离合器踏板影响）
	var effective_friction :float= friction * max(0.0, 1.0 - clutch_pedal)
	var dv := engine_av - gearbox_av
	var abs_dv :float= abs(dv)

	if abs_dv < dv_threshold:
		if abs(engine_torque) <= effective_friction:
			return [engine_torque, true]   # 黏滞
		else:
			return [effective_friction * sign(engine_torque), false]
	else:
		return [effective_friction * sign(dv), false]
