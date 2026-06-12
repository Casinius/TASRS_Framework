extends MeshInstance3D

@export var curve_path : Path3D
@export var cross_section : PackedVector2Array = [Vector2(-1,0), Vector2(1,0)]  # 默认一条线
@export var uv_scale : float = 1.0

func _ready():
	if not curve_path:
		return
	var st = SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	
	var curve = curve_path.curve
	var samples = 100  # 曲线细分段数
	
	# 沿曲线生成顶点和索引 (此处仅示意，完整实现需处理旋转和偏移)
	for i in range(samples):
		var t = float(i) / (samples - 1)
		var pos = curve.sample_baked(t * curve.get_baked_length())
		# 对每个截面点生成两个三角形...
		# （代码略，完整实现需要几百行）
	
	# 关键：自动生成平滑法线
	st.generate_normals()
	st.generate_tangents()
	mesh = st.commit()
