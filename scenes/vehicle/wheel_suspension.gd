class_name RaycastSuspension
extends RayCast3D

############# Choose what tire formula to use #############
var tire_model: BaseTireModel 

############# Suspension stuff #############
var spring_length = 0.2
var spring_stiffness = 45.0
var bump = 3.5
var rebound = 4.0
var anti_roll = 0.0

var spring_load_mm:float = 0
var prev_spring_load_mm:float = 0
var spring_speed_mm_per_seconds:float = 0
var spring_load_newton:float = 0

############# Tire stuff #############
var wheel_mass = 15.0
var tire_radius = 0.3
var tire_width = 0.2
var ackermann = 0.15

var tire_wear: float = 0.0

var surface_mu = 1.0
var y_force: float = 0.0

var wheel_inertia: float = 0.0
var spin: float = 0.0
var z_vel: float = 0.0
var local_vel := Vector3.ZERO

var rolling_resistance: float = 0.0 #Vector2 = Vector2.ZERO
var rol_res_surface_mul: float = 0.02

var force_vec = Vector3.ZERO
var slip_vec: Vector2 = Vector2.ZERO
var prev_pos: Vector3 = Vector3.ZERO

var spring_curr_length: float = spring_length

@onready var car = $'..' #Get the parent node as car
@export var wheelmesh:Node3D
@export var epsilon:float = 0.1
var max_extension_force:float=spring_stiffness * (spring_length * 1000)

func _ready() -> void:
	wheel_inertia = 0.5 * wheel_mass * pow(tire_radius, 2)
	set_target_position(Vector3.DOWN * (spring_length + tire_radius))


func set_params(params: WheelSuspensionParameters):
	tire_model =  params.tire_model
	spring_length = params.spring_length
	spring_stiffness = params.spring_stiffness
	bump = params.bump
	rebound = params.rebound
	wheel_mass = params.wheel_mass
	tire_radius = params.tire_model.tire_radius
	tire_width = params.tire_model.tire_width
	ackermann = params.ackermann
	anti_roll = params.anti_roll
	
	wheel_inertia = 0.5 * wheel_mass * pow(tire_radius, 2)
	set_target_position(Vector3.DOWN * (spring_length + tire_radius))


# Move back to physics process when physics interpolation comes to godot4
func _process(delta: float) -> void:
	
	var spin_treshold := 10.0
	var ambient_temp := 20.0
	
	# 【新增】瞬态滑移模型
	var raw_slip := slip_vec

	if abs(spin) > spin_treshold or abs(z_vel) > 1.0:
		tire_wear = tire_model.update_tire_wear(delta, slip_vec, y_force, surface_mu)
	if is_colliding() and y_force > 0:
		tire_model.update_tire_temp(slip_vec, y_force, local_vel.length(), surface_mu, ambient_temp, delta)
	
	wheelmesh.rotate_x(wrapf(-spin * delta,0, TAU))
	wheelmesh.position.y = -spring_curr_length


func _physics_process(delta: float) -> void:
	var spin_treshold := 10.0
	var ambient_temp := 20.0
	if abs(spin) > spin_treshold or abs(z_vel) > 1.0:
		tire_wear = tire_model.update_tire_wear(delta, slip_vec, y_force, surface_mu)
	if is_colliding() and y_force>0:
		tire_model.update_tire_temp(slip_vec, y_force, local_vel.length(), surface_mu, ambient_temp, delta)
	
	


# 在类顶部添加（替换原来的 mm 版本）
var prev_spring_load_m: float = 0.0


func apply_forces(opposite_comp, delta):
	force_vec = Vector3.ZERO
	slip_vec = Vector2.ZERO
	
	var surface
	
	if is_colliding():
		var collider :Node3D = self.get_collider()
		if collider:
			var collider_group = collider.get_groups()
			if collider_group.size() > 0:
				surface = collider_group[0]
		
		if surface:
			surface_mu = 1.0
			rol_res_surface_mul = 0.02
			if surface == "Tarmac":
				surface_mu = 0.85 
				rol_res_surface_mul = 0.01
			elif surface == "Gravel":
				surface_mu = 0.6
				rol_res_surface_mul = 0.03
			elif surface == "Grass":
				surface_mu = 0.55  
				rol_res_surface_mul = 0.025
			elif surface == "Snow":
				surface_mu = 0.4
				rol_res_surface_mul = 0.035
		
		spring_curr_length = get_collision_point().distance_to(global_transform.origin) - tire_radius
	else:
		spring_curr_length = spring_length
	
	spring_load_mm = (spring_length - spring_curr_length) * 1000
	spring_speed_mm_per_seconds = (spring_load_mm - prev_spring_load_mm) / delta
	prev_spring_load_mm = spring_load_mm
	spring_load_newton = spring_load_mm * spring_stiffness
	
	if spring_speed_mm_per_seconds >= 0:
		spring_load_newton += spring_speed_mm_per_seconds * bump
	else:
		spring_load_newton += spring_speed_mm_per_seconds * rebound
	
	y_force = spring_load_newton
	y_force = clamp(y_force, 0, max_extension_force)
	
	var min_speed_for_slip = 1.5
	
	if is_colliding():
		var normal = get_collision_normal()
		var contact_point = get_collision_point()
		
		var car_body_vel = car.linear_velocity
		var car_body_ang_vel = car.angular_velocity
		var wheel_offset = contact_point - car.global_transform.origin
		var contact_world_vel = car_body_vel + car_body_ang_vel.cross(wheel_offset)
		
		var wheel_forward_world = -global_transform.basis.z
		var wheel_right_world = global_transform.basis.x
		
		var ground_tangent_forward = wheel_forward_world - normal * wheel_forward_world.dot(normal)
		if ground_tangent_forward.length_squared() > 0.001:
			ground_tangent_forward = ground_tangent_forward.normalized()
		else:
			ground_tangent_forward = wheel_forward_world
		
		var ground_tangent_right = wheel_right_world - normal * wheel_right_world.dot(normal)
		if ground_tangent_right.length_squared() > 0.001:
			ground_tangent_right = ground_tangent_right.normalized()
		else:
			ground_tangent_right = wheel_right_world
		
		var v_long = contact_world_vel.dot(ground_tangent_forward)
		var v_lat = contact_world_vel.dot(ground_tangent_right)
		
		if abs(v_long) > min_speed_for_slip:
			slip_vec.x = atan2(-v_lat, abs(v_long))
		else:
			var characteristic_speed = 5.0
			slip_vec.x = clampf(-v_lat / characteristic_speed, -0.5, 0.5)
		
		var slip_ratio = (v_long - tire_radius * spin) / max(abs(v_long), epsilon)
		slip_vec.y = clampf(slip_ratio, -1.0, 1.0)
		
		if spring_load_mm != 0:
			y_force += anti_roll * (spring_load_mm - opposite_comp)
		
		force_vec = tire_model.update_tire_forces(slip_vec, y_force, surface_mu)
		
		var contact_local = contact_point - car.global_transform.origin
		
		rolling_resistance = rol_res_surface_mul * y_force
		
		car.apply_force(normal * y_force, contact_local)
		car.apply_force(global_transform.basis.x * force_vec.x, contact_local)
		car.apply_force(global_transform.basis.z * force_vec.y, contact_local)
		if force_vec.z != 0:
			car.apply_torque(Vector3(0, force_vec.z * 0.5, 0))
		
		return spring_load_mm
	else:
		var damping_torque = 2.0
		spin -= sign(spin) * delta * damping_torque / wheel_inertia
		return 0.0

func apply_torque(drive_torque, brake_torque, drive_inertia, delta):
	var prev_spin = spin
	var net_torque = force_vec.y * tire_radius
	net_torque += drive_torque
	if abs(spin) < 5 and brake_torque > abs(net_torque):
		spin = 0
	else:
		#net_torque -= (brake_torque + rolling_resistance) * sign(spin)
		net_torque -= (brake_torque + rolling_resistance * tire_radius) * sign(spin)
		spin += delta * net_torque / (wheel_inertia + drive_inertia)

	if drive_torque * delta == 0:
		return 0.5
	else:
		return (spin - prev_spin) * (wheel_inertia + drive_inertia) / (drive_torque * delta)


func set_spin(value):
	spin = value 


func get_spin():
	return spin


func get_reaction_torque():
	return force_vec.y * tire_radius


func steer(input, max_steer):
	rotation.y = max_steer * (input + (1 - cos(input * 0.5 * PI)) * ackermann)
	rotation.y = clamp(rotation.y,-max_steer,max_steer);
