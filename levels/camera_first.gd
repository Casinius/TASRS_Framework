extends Camera3D

# ----- 导出参数（带类型） -----
@export var vehicle: BaseCar                     # 关联车辆
@export var base_fov: float = 70.0              # 基础FOV
@export var look_smoothness: float = 5.0        # 动态响应平滑速度

# 俯仰（Pitch）参数
@export var pitch_angle_max: float = 2.5        # 最大俯仰角度（度）
@export var pitch_displacement: float = 0.05    # 头部前后位移幅度

# 侧倾（Roll）参数
@export var roll_angle_max: float = 3.0         # 最大侧倾角度（度）
@export var roll_displacement: float = 0.03     # 头部横向位移幅度

# FOV 速度响应
@export var fov_change_range: float = 3.0       # FOV随速度变化范围（度）
@export var fov_reference_speed: float = 200.0  # 参考车速（km/h）

# 颠簸参数
@export var bump_intensity: float = 0.02        # 颠簸位移幅度
@export var bump_frequency: float = 20.0        # 噪声频率

# ----- 内部状态（带类型） -----
var current_pitch: float = 0.0
var current_roll: float = 0.0
var current_fov_offset: float = 0.0
var head_offset: Vector3 = Vector3.ZERO

# 加速度缓存
var last_velocity: Vector3 = Vector3.ZERO

# 基础位置（在 _ready 中保存）
var base_position: Vector3 = Vector3.ZERO


func _ready() -> void:
	# 保存初始位置（作为子节点时，相对车辆的局部坐标）
	base_position = position
	fov = base_fov


func _physics_process(delta: float) -> void:
	if not vehicle:
		return

	# ----- 1. 获取车辆物理数据（需要车辆暴露 linear_velocity） -----
	var speed: float = vehicle.speedo                         # km/h
	var velocity: Vector3 = vehicle.linear_velocity          # 假设 vehicle 有该属性
	var local_vel: Vector3 = vehicle.global_transform.basis.inverse() * velocity
	var accel: Vector3 = (velocity - last_velocity) / delta
	var local_accel: Vector3 = vehicle.global_transform.basis.inverse() * accel
	last_velocity = velocity

	# 纵向加速度（向前为正）– Godot 局部 -Z 为前进方向
	var forward_accel: float = -local_accel.z
	# 横向加速度（向右为正）
	var lateral_accel: float = local_accel.x

	# ----- 2. 计算目标俯仰（加速抬头，刹车低头） -----
	var target_pitch: float = clamp(forward_accel / 9.8, -1.0, 1.0) * pitch_angle_max
	target_pitch = deg_to_rad(target_pitch)

	# ----- 3. 计算目标侧倾（右转左倾，左转右倾） -----
	var target_roll: float = -clamp(lateral_accel / 9.8, -1.0, 1.0) * roll_angle_max
	target_roll = deg_to_rad(target_roll)

	# ----- 4. 头部位移 -----
	var target_head_z: float = -clamp(forward_accel / 9.8, -1.0, 1.0) * pitch_displacement
	var target_head_x: float = clamp(lateral_accel / 9.8, -1.0, 1.0) * roll_displacement

	# ----- 5. 平滑插值（低通滤波） -----
	var lerp_factor: float = look_smoothness * delta
	current_pitch = lerp(current_pitch, target_pitch, lerp_factor)
	current_roll = lerp(current_roll, target_roll, lerp_factor)
	head_offset.x = lerp(head_offset.x, target_head_x, lerp_factor)
	head_offset.z = lerp(head_offset.z, target_head_z, lerp_factor)

	# ----- 6. 应用旋转（局部坐标） -----
	rotation.x = current_pitch
	rotation.z = current_roll
	# rotation.y 通常保持 0，如需跟随方向盘可另行实现

	# ----- 7. 应用位移（局部坐标） -----
	position.x = base_position.x + head_offset.x
	position.z = base_position.z + head_offset.z

	# ----- 8. 颠簸（高频噪声） -----
	# 简单模拟：使用正弦组合或Perlin，这里用随机数 + 帧率补偿
	
	
	#var bump_offset: float = bump_intensity * (randf() - 0.5) * bump_frequency * delta
	#position.y += bump_offset   # 注意：Y轴会累积漂移，建议使用正弦波
	# 更稳定的方式：用正弦波
	var bump_time: float = Time.get_ticks_msec() / 1000.0
	position.y = base_position.y + bump_intensity * sin(bump_time * bump_frequency) * (vehicle.speedo/800)

	# ----- 9. FOV 随速度变化（高速缩小） -----
	var speed_factor: float = clamp(speed / fov_reference_speed, 0.0, 1.0)
	var target_fov_offset: float = -speed_factor * fov_change_range
	current_fov_offset = lerp(current_fov_offset, target_fov_offset, lerp_factor)
	fov = base_fov + current_fov_offset

	# ----- 10. 可选：极微小高频抖动（旋转） -----
	# rotation.x += (randf() - 0.5) * 0.0005
	# rotation.z += (randf() - 0.5) * 0.0005
