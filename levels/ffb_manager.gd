extends FFBManager
@export var node:BaseCar;

var global_timer := 0;
@export var mod_engine_factor_hz = 40;
@export var ethta_car_body:float = 0.01;
@export var SmoothingFactor := 120.0;

@export var max_autoforce = 1800;

@export var lowspeed_ramp_speed := 3.0 

var prev_fz_fr := 0.0;
var prev_fz_fl := 0.0;
var prev_align_torque:=0.0;
var LowSpeed_Force := 0.0;
var max_lowspeed := 10.0;
var low_speed_dir :=0.0;
var prev_steering := 0.0;

var key_effect_const := 0;
#var key_effect_const_susp := 0;
var key_tire_fric:=0;
func _ready():
	init_sdl()
	key_effect_const = new_effect_constforce(0,1,0,0);
	#key_effect_const_susp = new_effect_constforce(0,1,0,0);
	start_effect(key_effect_const);
	key_tire_fric = new_effect_condition(1024,0,0,0,0,0,0,0,0);
	start_effect(key_effect_const);
	#start_effect(key_effect_const_susp)
	#print_info()
	#stop_effect()
	#constant_force(3000, 1, 0, 0)  # 启动一次，永久播放

func _physics_process(delta):
	var safe_speed :float= abs(node.speedo)
	
	# === 1. 回正力（AutoCenter）===
	const trail_length := 0.05
	var align_torque_fl :float= node.wheel_fl.force_vec.x * trail_length
	var align_torque_fr :float= node.wheel_fr.force_vec.x * trail_length
	var TqAlign := align_torque_fl + align_torque_fr
	var rear_fy :float= node.wheel_bl.force_vec.x + node.wheel_br.force_vec.x
	
	# 软限幅（增大阈值）
	const SoftenFactor := 1200.0
	var AutoCenterForce :float= TqAlign * SoftenFactor / (abs(TqAlign) + SoftenFactor)
	AutoCenterForce += rear_fy * 0.5
	
	# 低通滤波（一阶）
	var alpha := 1.0 / (1.0 + SmoothingFactor / 100.0)   # 约0.455
	AutoCenterForce = AutoCenterForce * alpha + prev_align_torque * (1.0 - alpha)
	prev_align_torque = AutoCenterForce
	
	# 速度因子：低速时衰减
	var autocenter_speed_factor :float= clamp((safe_speed - 1.0) / 20.0, 0.0, 1.0)
	var final_autocenter :float= AutoCenterForce * autocenter_speed_factor
	
	# === 2. 低速阻尼力（Parking/Damping）===
	var steer_center := node.steering_amount * node.car_params.max_steer
	var steer_velocity :float= (steer_center - prev_steering) / delta   # 转向角速度
	
	# 方向与转向速度相反（阻尼）
	var low_speed_dir := 0.0
	if abs(steer_velocity) > 0.01:
		var target :float= -sign(steer_velocity) * 7.0
		low_speed_dir = move_toward(low_speed_dir, target, lowspeed_ramp_speed * delta)
	else:
		low_speed_dir = move_toward(low_speed_dir, 0.0, lowspeed_ramp_speed * 0.5 * delta)
	low_speed_dir = clamp(low_speed_dir, -7.0, 7.0)
	
	# 速度衰减（随速度升高而减小）
	var speed_attenuation := 1.0 / (sqrt(safe_speed) + 0.5)
	var raw_low_force := (10000.0 / 8.0) * low_speed_dir * speed_attenuation
	
	# 平滑速度过渡因子（0~10 m/s 渐变）
	var low_speed_factor := smoothstep(10.0, 0.0, safe_speed)   # 速度越低值越大
	var final_lowspeed := raw_low_force * low_speed_factor
	
	# === 3. 合并总力 ===
	var total_force := final_autocenter + final_lowspeed
	
	# === 4. 映射到设备（tanh 非线性压缩）===
	var normalized := tanh(total_force / max_autoforce)
	var device_force := int(normalized * 32765.0)   # 直接映射到±32765
	
	update_effect_constforce(key_effect_const, device_force, 1)
	
	# === 5. 更新上一帧转向角（始终更新）===
	prev_steering = steer_center

# 自定义平滑阶跃函数
func smoothstep(edge0: float, edge1: float, x: float) -> float:
	var t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0)
	return t * t * (3.0 - 2.0 * t)
	
func _exit_tree():
	stop_effect(key_tire_fric);
	stop_effect(key_effect_const);
	deinit_sdl();
