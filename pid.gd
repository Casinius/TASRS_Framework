class_name PIDController
extends RefCounted

var kp: float = 0.0
var ki: float = 0.0
var kd: float = 0.0
var integral: float = 0.0
var prev_error: float = 0.0
var output_min: float = -INF
var output_max: float = INF
var integral_clamp: float = INF

func _init(p: float, i: float, d: float):
	kp = p
	ki = i
	kd = d

func update(setpoint: float, measurement: float, dt: float) -> float:
	var error = setpoint - measurement
	integral += error * dt
	# 抗积分饱和
	integral = clamp(integral, -integral_clamp, integral_clamp)
	var derivative = (error - prev_error) / dt if dt > 0 else 0.0
	prev_error = error
	var output = kp * error + ki * integral + kd * derivative
	return clamp(output, output_min, output_max)

func reset():
	integral = 0.0
	prev_error = 0.0
