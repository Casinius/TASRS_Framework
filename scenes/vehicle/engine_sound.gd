extends Node
class_name SingleEngineSound

# ========== 可导出参数 ==========
@export var engine_audio_stream: AudioStream   # 您的引擎循环音效（WAV/OGG，建议已做循环）
@export var max_engine_rpm: float = 7000.0
@export var rpm_idle: float = 800.0
@export var rpm_redline: float = 7000.0

# 音高映射范围（建议 0.6 ~ 1.4，避免过度失真）
@export var min_pitch: float = 0.6
@export var max_pitch: float = 1.4

# 平滑速度（值越大音高跟随越快，建议 5~12）
@export var smooth_speed: float = 8.0

# 淡入淡出时间（秒）
@export var fade_time: float = 0.3

# ========== 内部变量 ==========
var _player: AudioStreamPlayer
var _current_rpm: float = 0.0
var _target_rpm: float = 0.0
var _is_engine_on: bool = false
var _is_fading: bool = false

# ========== 初始化 ==========
func _ready():
	# 创建播放器
	_player = AudioStreamPlayer.new()
	add_child(_player)
	
	# 如果音频流自带循环，可在此设置；否则用信号重播
	if engine_audio_stream:
		_player.stream = engine_audio_stream
		# 确保循环播放（两种方式任选其一）
		# 方法1：在导入音频时勾选“Loop”属性（推荐）
		# 方法2：代码手动循环
		_player.finished.connect(_on_audio_finished)
	
	# 初始静音
	_player.volume_db = -80.0
	_current_rpm = rpm_idle
	_target_rpm = rpm_idle

# 音频结束回调（如果没开启循环，手动重播）
func _on_audio_finished():
	if _is_engine_on and not _player.playing:
		_player.play()

# ========== 核心更新函数（每帧调用） ==========
func update_engine_sound(delta: float, current_rpm: float):
	# 1. 限制输入转速（允许轻微超转）
	_target_rpm = clamp(current_rpm, 0.0, rpm_redline * 1.05)
	
	# 2. 引擎启停判断
	if _target_rpm <= 0.1 and _is_engine_on:
		_stop_engine_sound()
		return
	elif _target_rpm > rpm_idle * 0.5 and not _is_engine_on and not _is_fading:
		_start_engine_sound()
	
	if not _is_engine_on:
		return
	
	# 3. 平滑过渡转速（消除突变）
	_current_rpm = lerp(_current_rpm, _target_rpm, 1.0 - exp(-delta * smooth_speed))
	_current_rpm = clamp(_current_rpm, 0.0, rpm_redline * 1.1)
	
	# 4. 计算归一化转速（0~1）
	var norm_rpm = (_current_rpm - rpm_idle) / (rpm_redline - rpm_idle)
	norm_rpm = clamp(norm_rpm, 0.0, 1.0)
	
	# 5. ★ 音高映射（线性映射到 min_pitch ~ max_pitch）★
	var target_pitch = min_pitch + norm_rpm * (max_pitch - min_pitch)
	# 再对音高做一次平滑（防止高频小抖动）
	_player.pitch_scale = lerp(_player.pitch_scale, target_pitch, 0.2)  # 额外平滑
	
	# 6. ★ 音量动态补偿（随转速升高适当增大音量）★
	# 例如：怠速 -10dB，红线 0dB
	var volume_db = -10.0 + norm_rpm * 10.0
	_player.volume_db = clamp(volume_db, -20.0, 0.0)

# ========== 启动引擎（淡入） ==========
func _start_engine_sound():
	if _is_engine_on:
		return
	_is_engine_on = true
	_is_fading = false
	
	# 设置流（如果尚未设置）
	if _player.stream == null:
		_player.stream = engine_audio_stream
	
	# 淡入
	var tween = create_tween()
	_player.volume_db = -80.0
	_player.play()
	tween.tween_property(_player, "volume_db", 0.0, fade_time)

# ========== 停止引擎（淡出） ==========
func _stop_engine_sound(fade_time_custom: float = -1.0):
	if not _is_engine_on or _is_fading:
		return
	
	_is_engine_on = false
	_is_fading = true
	
	var fade_duration = fade_time_custom if fade_time_custom > 0 else fade_time
	
	var tween = create_tween()
	tween.tween_property(_player, "volume_db", -80.0, fade_duration)
	tween.tween_callback(_player.stop)
	tween.tween_callback(func(): _is_fading = false)

# ========== 对外公开的停止接口（可手动调用，如熄火） ==========
func stop_engine(fade_seconds: float = 0.5):
	_stop_engine_sound(fade_seconds)
