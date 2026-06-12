extends Camera3D

var global_time :=100;
var origin_x:=self.position.x;
var origin_fov:=self.fov;
var origin_rotx = self.rotation.x;
@export var node:BaseCar;
func _ready():
	origin_x=self.position.x;
	pass
func _physics_process(delta):
	
	#if(global_time==0):global_time=100;
	#global_time=global_time-1;
	#if(global_time==51):
	var speed_ms: int= node.speedo;
	#var to_tanslate := Vector3(-node.rotation.x*10*(speed_ms/6),0,0).normalized()/5000;
	#self.translate(to_tanslate);
	var to_rotate :=Vector3(node.rotation.x*10*(speed_ms),0,0).normalized()/5000;
	self.rotation.x=to_rotate.x;
	self.rotation.y = self.rotation.y*(speed_ms/60)*(Input.get_axis("Brake","Throttle")/100);

	#self.position.x = clamp(self.position.x,origin_x-0.05,origin_x+0.05);
	self.fov = clamp(Input.get_axis("Brake","Throttle")/100*(speed_ms/60),-10.0,10.0)+self.fov;
	self.fov=self.fov-0.05*(-Input.get_axis("Brake","Throttle"))*speed_ms/60;
	
	self.fov = clamp(self.fov,origin_fov-10,origin_fov+10); 
	
	pass
