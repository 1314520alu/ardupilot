-- Transwing fold-angle estimator, dynamic mix prototype, and transition guard.
-- SITL-first script: motor mix outputs stay isolated unless TW_LOG_ONLY=0.

local SCRIPT_NAME = "TW-DYNMIX"
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_WARNING = 4

local TABLE_KEY = 91
assert(param:add_table(TABLE_KEY, "TW_", 28), "could not add TW_ parameter table")

assert(param:add_param(TABLE_KEY, 1, "ENABLE", 1), "could not add TW_ENABLE")
assert(param:add_param(TABLE_KEY, 2, "LOG_ONLY", 1), "could not add TW_LOG_ONLY")
assert(param:add_param(TABLE_KEY, 3, "FOLD_CH", 10), "could not add TW_FOLD_CH")
assert(param:add_param(TABLE_KEY, 4, "PWM_FW", 2000), "could not add TW_PWM_FW")
assert(param:add_param(TABLE_KEY, 5, "PWM_Q", 1000), "could not add TW_PWM_Q")
assert(param:add_param(TABLE_KEY, 6, "RATE_UP", 4.5), "could not add TW_RATE_UP")
assert(param:add_param(TABLE_KEY, 7, "RATE_DN", 4.5), "could not add TW_RATE_DN")
assert(param:add_param(TABLE_KEY, 8, "TIMEOUT", 25), "could not add TW_TIMEOUT")
assert(param:add_param(TABLE_KEY, 9, "OUT_GAIN", 0.2), "could not add TW_OUT_GAIN")
assert(param:add_param(TABLE_KEY, 10, "THR", 0), "could not add TW_THR")
assert(param:add_param(TABLE_KEY, 11, "ROLL", 0), "could not add TW_ROLL")
assert(param:add_param(TABLE_KEY, 12, "PITCH", 0), "could not add TW_PITCH")
assert(param:add_param(TABLE_KEY, 13, "YAW", 0), "could not add TW_YAW")
assert(param:add_param(TABLE_KEY, 14, "GUARD", 1), "could not add TW_GUARD")
assert(param:add_param(TABLE_KEY, 15, "SAFE_MIN", 55), "could not add TW_SAFE_MIN")
assert(param:add_param(TABLE_KEY, 16, "BLEND_AS", 13), "could not add TW_BLEND_AS")
assert(param:add_param(TABLE_KEY, 17, "FW_AS", 18), "could not add TW_FW_AS")
assert(param:add_param(TABLE_KEY, 18, "ATT_DANG", 55), "could not add TW_ATT_DANG")
assert(param:add_param(TABLE_KEY, 19, "DESC_DANG", -8), "could not add TW_DESC_DANG")
assert(param:add_param(TABLE_KEY, 20, "SAT_PWM", 1980), "could not add TW_SAT_PWM")
assert(param:add_param(TABLE_KEY, 21, "ACCEL_MIN", 55), "could not add TW_ACCEL_MIN")
assert(param:add_param(TABLE_KEY, 22, "ATT_ABORT", 35), "could not add TW_ATT_ABORT")
assert(param:add_param(TABLE_KEY, 23, "MIX_MODE", 0), "could not add TW_MIX_MODE")
assert(param:add_param(TABLE_KEY, 24, "INPUT_SRC", 1), "could not add TW_INPUT_SRC")
assert(param:add_param(TABLE_KEY, 25, "GUARD_FBWA", 1), "could not add TW_GUARD_FBWA")
assert(param:add_param(TABLE_KEY, 26, "MIX_BLEND", 0), "could not add TW_MIX_BLEND")
assert(param:add_param(TABLE_KEY, 27, "GUARD_MS", 1000), "could not add TW_GUARD_MS")
assert(param:add_param(TABLE_KEY, 28, "BLEND_MIN", 30), "could not add TW_BLEND_MIN")

local P = {}
for _, name in ipairs({
  "ENABLE", "LOG_ONLY", "FOLD_CH", "PWM_FW", "PWM_Q", "RATE_UP", "RATE_DN",
  "TIMEOUT", "OUT_GAIN", "THR", "ROLL", "PITCH", "YAW", "GUARD", "SAFE_MIN",
  "BLEND_AS", "FW_AS", "ATT_DANG", "DESC_DANG", "SAT_PWM", "ACCEL_MIN", "ATT_ABORT",
  "MIX_MODE", "INPUT_SRC", "GUARD_FBWA", "MIX_BLEND", "GUARD_MS", "BLEND_MIN"
}) do
  P[name] = Parameter("TW_" .. name)
end

local FACTOR_TABLE = {
  {
    theta = 0,
    motors = {
      { name = "M1", throttle = 0, roll = 0.000000, pitch = -0.750000, yaw_force = -0.326531, yaw_drag = 1 },
      { name = "M2", throttle = 0, roll = 0.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 0, roll = 0.000000, pitch = -0.750000, yaw_force = 0.326531, yaw_drag = -1 },
      { name = "M4", throttle = 0, roll = 0.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 15,
    motors = {
      { name = "M1", throttle = 1, roll = -0.353266, pitch = -0.691231, yaw_force = -0.353266, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.353266, pitch = -0.691231, yaw_force = 0.353266, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 30,
    motors = {
      { name = "M1", throttle = 1, roll = -0.430762, pitch = 1.000000, yaw_force = -0.430762, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = 0.213987, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.430762, pitch = 1.000000, yaw_force = 0.430762, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = 0.213987, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 45,
    motors = {
      { name = "M1", throttle = 1, roll = -0.562500, pitch = 1.000000, yaw_force = -0.562500, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -0.100000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.562500, pitch = 1.000000, yaw_force = 0.562500, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -0.100000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 60,
    motors = {
      { name = "M1", throttle = 1, roll = -0.749689, pitch = 1.000000, yaw_force = -0.749689, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -0.536819, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.749689, pitch = 1.000000, yaw_force = 0.749689, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -0.536819, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 75,
    motors = {
      { name = "M1", throttle = 1, roll = -0.957651, pitch = 1.000000, yaw_force = -0.957651, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -0.869630, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.957651, pitch = 1.000000, yaw_force = 0.957651, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -0.869630, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 90,
    motors = {
      { name = "M1", throttle = 1, roll = -1.000000, pitch = 1.000000, yaw_force = 0.000000, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 0.938776, pitch = -1.000000, yaw_force = 0.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 1.000000, pitch = 1.000000, yaw_force = 0.000000, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -0.938776, pitch = -1.000000, yaw_force = 0.000000, yaw_drag = -1 },
    },
  },
}

local PHASE = { HOVER = 0, ACCEL = 1, BLEND = 2, FIXED_WING = 3 }
local RISK = { OK = 0, WARN = 1, DANGER = 2 }
local ACTION = { ALLOW = 0, HOLD_THETA = 1, ABORT_TO_Q = 2 }
local MIX_MODE_OBSERVE = 0
local MIX_MODE_MIRROR = 1
local MIX_MODE_CONTROL = 2
local INPUT_SRC_PARAM = 0
local INPUT_SRC_RC = 1
local MODE_FBWA = 5
local MODE_FBWB = 6
local MODE_QSTABILIZE = 17

local theta_est = 0
local theta_target = 0
local last_ms = millis():tofloat()
local boot_ms = last_ms
local target_changed_ms = last_ms
local last_target = nil
local warned_timeout = false
local warned_guard = false
local last_guard_reason = nil
local announced = false
local last_ann_mix = nil
local last_ann_log = nil
local climb_filt = 0
local climb_filt_init = false
local descent_exceed_start_ms = nil
local motors_registered = false
local motors_dynamic_active = false
local warned_no_motors_dynamic = false

-- Motor test order A,C,D,B for Transwing M1-M4 on Quad X dynamic matrix.
local MOTOR_TEST_ORDER = { 1, 3, 4, 2 }

local CLIMB_FILTER_TAU_S = 0.5
local DESC_SUSTAIN_S = 0.4
local DESC_SEVERE_SCALE = 1.5
local BOOT_GUARD_GRACE_S = 15
local BOOT_GUARD_MAX_AS = 3

local function clamp(value, min_value, max_value)
  if value < min_value then return min_value end
  if value > max_value then return max_value end
  return value
end

local function lerp(a, b, ratio)
  return a + (b - a) * ratio
end

local function pwm_to_theta_target(pwm, pwm_fw, pwm_q)
  if pwm == nil or pwm_fw == pwm_q then return theta_target end
  local ratio = clamp((pwm - pwm_fw) / (pwm_q - pwm_fw), 0, 1)
  return ratio * 90
end

local function theta_to_pwm(theta, pwm_fw, pwm_q)
  return math.floor(pwm_fw + clamp(theta, 0, 90) / 90 * (pwm_q - pwm_fw) + 0.5)
end

local function step_theta_estimate(theta, target, dt, rate_up, rate_dn)
  local delta = target - theta
  if delta == 0 or dt <= 0 then return theta end
  local rate = delta > 0 and rate_up or rate_dn
  local max_step = math.abs(rate * dt)
  if math.abs(delta) <= max_step then return target end
  return theta + (delta > 0 and max_step or -max_step)
end

local function interpolate_factors(theta)
  if theta <= FACTOR_TABLE[1].theta then return FACTOR_TABLE[1].motors end
  if theta >= FACTOR_TABLE[#FACTOR_TABLE].theta then return FACTOR_TABLE[#FACTOR_TABLE].motors end

  local lower = FACTOR_TABLE[1]
  local upper = FACTOR_TABLE[#FACTOR_TABLE]
  for i = 2, #FACTOR_TABLE do
    if FACTOR_TABLE[i].theta >= theta then
      lower = FACTOR_TABLE[i - 1]
      upper = FACTOR_TABLE[i]
      break
    end
  end

  local ratio = (theta - lower.theta) / (upper.theta - lower.theta)
  local out = {}
  for i = 1, 4 do
    local lo = lower.motors[i]
    local hi = upper.motors[i]
    out[i] = {
      name = lo.name,
      throttle = lerp(lo.throttle, hi.throttle, ratio),
      roll = lerp(lo.roll, hi.roll, ratio),
      pitch = lerp(lo.pitch, hi.pitch, ratio),
      yaw_force = lerp(lo.yaw_force, hi.yaw_force, ratio),
      yaw_drag = lerp(lo.yaw_drag, hi.yaw_drag, ratio),
    }
  end
  return out
end

local function mix_to_pwm(factor, throttle, roll, pitch, yaw, gain)
  local min_pwm = 1100
  local hover_pwm = 1500
  local max_pwm = 1900
  local normalized = throttle * factor.throttle +
    gain * (roll * factor.roll + pitch * factor.pitch + yaw * (factor.yaw_force + factor.yaw_drag))
  return math.floor(clamp(hover_pwm + normalized * (max_pwm - hover_pwm), min_pwm, max_pwm) + 0.5)
end

local function phase_for_theta(theta)
  local blend_min = P.BLEND_MIN:get()
  local accel_min = P.ACCEL_MIN:get()
  if theta > 75 then return PHASE.HOVER end
  if theta >= accel_min then return PHASE.ACCEL end
  if theta >= blend_min then return PHASE.BLEND end
  return PHASE.FIXED_WING
end

local function read_airspeed()
  if ahrs == nil then return 0 end
  local ok, value = pcall(function() return ahrs:airspeed_estimate() end)
  if ok and value ~= nil then return value end
  return 0
end

local function read_attitude_deg()
  if ahrs == nil then return 0, 0 end
  local ok_roll, roll = pcall(function() return ahrs:get_roll() end)
  local ok_pitch, pitch = pcall(function() return ahrs:get_pitch() end)
  if not ok_roll or roll == nil then roll = 0 end
  if not ok_pitch or pitch == nil then pitch = 0 end
  return math.deg(roll), math.deg(pitch)
end

local function get_param_value(name, default)
  local ok, param_obj = pcall(function() return Parameter(name) end)
  if ok and param_obj ~= nil then
    local ok_get, value = pcall(function() return param_obj:get() end)
    if ok_get and value ~= nil then return value end
  end
  return default
end

local function norm_rc_pwm(pwm, min_pwm, max_pwm)
  if pwm == nil or pwm < 1 or max_pwm == min_pwm then return nil end
  return clamp((pwm - min_pwm) / (max_pwm - min_pwm), 0, 1)
end

local function norm_rc_stick(pwm, center, half_span)
  if pwm == nil or pwm < 1 or half_span <= 0 then return nil end
  return clamp((pwm - center) / half_span, -1, 1)
end

local function read_control_inputs()
  local throttle = clamp(P.THR:get(), 0, 1)
  local roll = clamp(P.ROLL:get(), -1, 1)
  local pitch = clamp(P.PITCH:get(), -1, 1)
  local yaw = clamp(P.YAW:get(), -1, 1)

  if math.floor(P.INPUT_SRC:get() + 0.5) ~= INPUT_SRC_RC then
    return throttle, roll, pitch, yaw
  end

  if rc == nil then return throttle, roll, pitch, yaw end

  local pwm_min = get_param_value("Q_M_PWM_MIN", 1000)
  local pwm_max = get_param_value("Q_M_PWM_MAX", 2000)
  local stick_half = math.max(get_param_value("RC1_DZ", 0) + 500, 100)

  local ok_thr, thr_pwm = pcall(function() return rc:get_pwm(3) end)
  if ok_thr then
    local normalized = norm_rc_pwm(thr_pwm, pwm_min, pwm_max)
    if normalized ~= nil then throttle = normalized end
  end

  local stick_channels = {
    { chan = 1, setter = function(v) roll = v end },
    { chan = 2, setter = function(v) pitch = v end },
    { chan = 4, setter = function(v) yaw = v end },
  }
  for _, mapping in ipairs(stick_channels) do
    local ok_pwm, stick_pwm = pcall(function() return rc:get_pwm(mapping.chan) end)
    if ok_pwm then
      local center = get_param_value("RC" .. mapping.chan .. "_TRIM", 1500)
      local normalized = norm_rc_stick(stick_pwm, center, stick_half)
      if normalized ~= nil then mapping.setter(normalized) end
    end
  end

  return throttle, roll, pitch, yaw
end

local function get_flight_mode()
  if vehicle == nil then return -1 end
  local ok, mode = pcall(function() return vehicle:get_mode() end)
  if ok and mode ~= nil then return mode end
  return -1
end

local function mode_allows_lua_mix(mode)
  if mode == MODE_FBWA then return false end
  if mode == MODE_FBWB then return true end
  if mode >= MODE_QSTABILIZE and mode <= 23 then return true end
  return false
end

local function can_takeover_motors(mix_mode, action, flight_mode, target, estimate)
  if P.LOG_ONLY:get() >= 0.5 then return false end
  if mix_mode ~= MIX_MODE_CONTROL then return false end
  if action ~= ACTION.ALLOW then return false end
  if not mode_allows_lua_mix(flight_mode) then return false end
  if math.abs(target - estimate) >= 3 then return false end
  return true
end

local function register_motors_dynamic()
  if motors_registered or Motors_dynamic == nil then
    return motors_registered
  end
  for i = 0, 3 do
    Motors_dynamic:add_motor(i, MOTOR_TEST_ORDER[i + 1])
  end
  motors_registered = true
  return true
end

local function apply_dynamic_motor_mix(factors)
  if Motors_dynamic == nil then
    return false
  end
  if not register_motors_dynamic() then
    return false
  end

  local factor_table = motor_factor_table()
  for i = 1, 4 do
    local f = factors[i]
    local idx = i - 1
    factor_table:roll(idx, f.roll)
    factor_table:pitch(idx, f.pitch)
    factor_table:yaw(idx, f.yaw_force + f.yaw_drag)
    factor_table:throttle(idx, f.throttle)
  end

  Motors_dynamic:load_factors(factor_table)
  if not Motors_dynamic:init(4) then
    return false
  end

  if motors ~= nil then
    pcall(function() motors:set_frame_string("TW DynMix") end)
  end
  return true
end

local function warn_missing_motors_dynamic()
  if warned_no_motors_dynamic then return end
  gcs:send_text(MAV_SEVERITY_WARNING,
    SCRIPT_NAME .. ": set Q_FRAME_CLASS=17 for Lua motor mix (Motors_dynamic)")
  warned_no_motors_dynamic = true
end

local function read_climb_rate()
  if ahrs == nil then return 0 end
  local ok, vel = pcall(function() return ahrs:get_velocity_NED() end)
  if ok and vel ~= nil then
    local ok_z, z = pcall(function() return vel:z() end)
    if ok_z and z ~= nil then return -z end
  end
  return 0
end

local function is_saturated(pwm, sat_pwm)
  -- Only treat upper PWM clamp as saturation. A lower bound of 1100 falsely
  -- flags Q_M_PWM_MIN (~1000) idle outputs during transition.
  local max_pwm = 0
  for i = 1, #pwm do
    if pwm[i] > max_pwm then max_pwm = pwm[i] end
    if pwm[i] >= sat_pwm then return 1 end
  end
  -- Differential floor: one motor at ESC minimum while another is near max.
  for i = 1, #pwm do
    if pwm[i] <= 1050 and max_pwm >= (sat_pwm - 50) then return 1 end
  end
  return 0
end

local function read_motor_pwm_fallback(fallback)
  local actual = {}
  local have_actual = false
  for i = 1, 4 do
    local value = SRV_Channels:get_output_pwm_chan(i - 1)
    if value ~= nil and value > 0 then
      actual[i] = value
      have_actual = true
    else
      actual[i] = fallback[i]
    end
  end
  if have_actual then return actual end
  return fallback
end

local function append_reason(parts, text)
  if text ~= nil and text ~= "" then
    parts[#parts + 1] = text
  end
end

local function build_guard_reason(target, airspeed, roll_deg, pitch_deg, climb_rate, saturation, descent_sustained)
  local reasons = {}
  local accel_min = P.ACCEL_MIN:get()
  local blend_min = P.BLEND_MIN:get()

  if target >= accel_min and target < 90 and airspeed < P.BLEND_AS:get() then
    append_reason(reasons, string.format("accel airspeed %.1f<%.1f", airspeed, P.BLEND_AS:get()))
  end
  if target < accel_min and target >= blend_min and airspeed < P.BLEND_AS:get() then
    append_reason(reasons, string.format("blend airspeed %.1f<%.1f", airspeed, P.BLEND_AS:get()))
  end
  if target < blend_min and airspeed < P.FW_AS:get() then
    append_reason(reasons, string.format("fixed-wing airspeed %.1f<%.1f", airspeed, P.FW_AS:get()))
  end
  if math.abs(roll_deg) > P.ATT_ABORT:get() or math.abs(pitch_deg) > P.ATT_ABORT:get() then
    append_reason(reasons, string.format("attitude roll=%.1f pitch=%.1f > abort %.1f",
      math.abs(roll_deg), math.abs(pitch_deg), P.ATT_ABORT:get()))
  elseif math.abs(roll_deg) > P.ATT_DANG:get() or math.abs(pitch_deg) > P.ATT_DANG:get() then
    append_reason(reasons, string.format("attitude roll=%.1f pitch=%.1f > danger %.1f",
      math.abs(roll_deg), math.abs(pitch_deg), P.ATT_DANG:get()))
  end
  if climb_rate < P.DESC_DANG:get() then
    append_reason(reasons, string.format("descent %.1f<%.1f", climb_rate, P.DESC_DANG:get()))
  end
  if descent_sustained and climb_rate < P.DESC_DANG:get() then
    append_reason(reasons, string.format("descent sustained %.1fs", DESC_SUSTAIN_S))
  end
  if saturation > 0.5 then
    append_reason(reasons, "saturation")
  end

  if #reasons == 0 then
    return "nominal"
  end
  return table.concat(reasons, "; ")
end

local function effective_mix_mode()
  local mode = math.floor(P.MIX_MODE:get() + 0.5)
  if mode < MIX_MODE_OBSERVE then mode = MIX_MODE_OBSERVE end
  if mode > MIX_MODE_CONTROL then mode = MIX_MODE_CONTROL end
  if P.LOG_ONLY:get() < 0.5 and mode == MIX_MODE_OBSERVE then
    return MIX_MODE_MIRROR
  end
  return mode
end

local function safe_min_for_phase(target, accel_min, blend_min)
  if target < blend_min then return blend_min end
  if target < accel_min then return accel_min end
  return target
end

local function should_abort_for_descent(target, airspeed, climb_rate, descent_sustained, attitude_abort, attitude_danger)
  local thresh = P.DESC_DANG:get()
  local severe = climb_rate < (thresh * DESC_SEVERE_SCALE)
  local blend_min = P.BLEND_MIN:get()
  local in_fw = target < blend_min and airspeed >= P.FW_AS:get()

  if in_fw then
    return severe or (descent_sustained and (attitude_abort or attitude_danger))
  end
  return severe or descent_sustained
end

local function evaluate_transition(theta, target, airspeed, roll_deg, pitch_deg, climb_rate, saturation, flight_mode, descent_sustained)
  local phase = phase_for_theta(theta)
  local risk = RISK.OK
  local action = ACTION.ALLOW
  local allowed = target
  local accel_min = P.ACCEL_MIN:get()
  local blend_min = P.BLEND_MIN:get()
  local attitude_abort = math.abs(roll_deg) > P.ATT_ABORT:get() or math.abs(pitch_deg) > P.ATT_ABORT:get()
  local attitude_danger = math.abs(roll_deg) > P.ATT_DANG:get() or math.abs(pitch_deg) > P.ATT_DANG:get()
  local fbwa_low_speed = P.GUARD_FBWA:get() > 0.5 and flight_mode == MODE_FBWA and airspeed < P.BLEND_AS:get()
  local descent_abort = should_abort_for_descent(
    target, airspeed, climb_rate, descent_sustained, attitude_abort, attitude_danger
  )

  if target >= accel_min and target < 90 and airspeed < P.BLEND_AS:get() then
    risk = RISK.WARN
  end
  if target < accel_min and target >= blend_min and airspeed < P.BLEND_AS:get() then
    risk = RISK.DANGER
  end
  if target < blend_min and airspeed < P.FW_AS:get() then
    risk = RISK.DANGER
  end
  if attitude_abort or attitude_danger then risk = RISK.DANGER end
  if climb_rate < P.DESC_DANG:get() then risk = RISK.DANGER end
  if saturation > 0.5 then risk = RISK.DANGER end

  if risk == RISK.DANGER and target < accel_min and target >= blend_min then
    action = ACTION.HOLD_THETA
    allowed = accel_min
  end
  if risk == RISK.DANGER and target < blend_min then
    action = ACTION.HOLD_THETA
    allowed = blend_min
  end
  local abort_trigger = attitude_abort or descent_abort or saturation > 0.5
  if risk == RISK.DANGER and target < 90 and abort_trigger then
    if fbwa_low_speed and attitude_abort and not descent_abort and saturation <= 0.5 then
      action = ACTION.HOLD_THETA
      allowed = math.max(target, safe_min_for_phase(target, accel_min, blend_min))
    else
      action = ACTION.ABORT_TO_Q
      allowed = 90
    end
  end

  return phase, risk, action, allowed, build_guard_reason(
    target, airspeed, roll_deg, pitch_deg, climb_rate, saturation, descent_sustained
  )
end

local function update()
  local now_ms = millis():tofloat()
  local dt = (now_ms - last_ms) * 0.001
  last_ms = now_ms

  if P.ENABLE:get() < 0.5 then return update, 100 end

  if not announced then
    local mix_mode = effective_mix_mode()
    local dyn_hint = Motors_dynamic ~= nil and "Motors_dynamic=ok" or "Motors_dynamic=missing"
    gcs:send_text(MAV_SEVERITY_INFO, string.format(
      "%s: running, mix_mode=%d log_only=%s blend=%.2f %s",
      SCRIPT_NAME, mix_mode, tostring(P.LOG_ONLY:get()), P.MIX_BLEND:get(), dyn_hint))
    announced = true
    last_ann_mix = mix_mode
    last_ann_log = P.LOG_ONLY:get()
  end

  local mix_mode_now = effective_mix_mode()
  local log_only_now = P.LOG_ONLY:get()
  if mix_mode_now ~= last_ann_mix or math.abs(log_only_now - (last_ann_log or -1)) > 0.01 then
    gcs:send_text(MAV_SEVERITY_INFO, string.format(
      "%s: running, mix_mode=%d log_only=%s blend=%.2f",
      SCRIPT_NAME, mix_mode_now, tostring(log_only_now), P.MIX_BLEND:get()))
    last_ann_mix = mix_mode_now
    last_ann_log = log_only_now
  end

  local fold_chan = math.floor(P.FOLD_CH:get() + 0.5)
  local fold_pwm = SRV_Channels:get_output_pwm_chan(fold_chan)
  theta_target = pwm_to_theta_target(fold_pwm, P.PWM_FW:get(), P.PWM_Q:get())

  if last_target == nil or math.abs(theta_target - last_target) > 0.5 then
    target_changed_ms = now_ms
    warned_timeout = false
    last_target = theta_target
  end

  theta_est = step_theta_estimate(theta_est, theta_target, dt, P.RATE_UP:get(), P.RATE_DN:get())

  local factors = interpolate_factors(theta_est)
  local throttle, roll, pitch, yaw = read_control_inputs()
  local gain = clamp(P.OUT_GAIN:get(), 0, 1)

  local pwm = {}
  for i = 1, 4 do pwm[i] = mix_to_pwm(factors[i], throttle, roll, pitch, yaw, gain) end

  local airspeed = read_airspeed()
  local roll_deg, pitch_deg = read_attitude_deg()
  local climb_raw = read_climb_rate()
  local alpha = clamp(dt / CLIMB_FILTER_TAU_S, 0, 1)
  if not climb_filt_init then
    climb_filt = climb_raw
    climb_filt_init = true
  else
    climb_filt = climb_filt + alpha * (climb_raw - climb_filt)
  end

  local desc_thresh = P.DESC_DANG:get()
  if climb_filt < desc_thresh then
    if descent_exceed_start_ms == nil then
      descent_exceed_start_ms = now_ms
    end
  else
    descent_exceed_start_ms = nil
  end
  local descent_sustained = descent_exceed_start_ms ~= nil and
    (now_ms - descent_exceed_start_ms) >= (DESC_SUSTAIN_S * 1000)

  local flight_mode = get_flight_mode()
  local motor_pwm = read_motor_pwm_fallback(pwm)
  local saturation = is_saturated(motor_pwm, P.SAT_PWM:get())
  local boot_elapsed_s = (now_ms - boot_ms) * 0.001
  local guard_enabled = P.GUARD:get() > 0.5
  if guard_enabled and boot_elapsed_s < BOOT_GUARD_GRACE_S and airspeed < BOOT_GUARD_MAX_AS then
    guard_enabled = false
  end

  local phase, risk, action, allowed_theta, guard_reason = evaluate_transition(
    theta_est, theta_target, airspeed, roll_deg, pitch_deg, climb_filt, saturation, flight_mode, descent_sustained
  )

  logger:write("TWNG", "Targ,Est,Fold,M1,M2,M3,M4,A1,A2,A3,A4", "fffffffffff",
    theta_target, theta_est, fold_pwm or 0,
    pwm[1], pwm[2], pwm[3], pwm[4],
    motor_pwm[1], motor_pwm[2], motor_pwm[3], motor_pwm[4])

  logger:write("TWTR", "Targ,Est,Allow,Phase,Risk,AS,Roll,Pitch,Climb,Sat,Act", "fffffffffff",
    theta_target, theta_est, allowed_theta, phase, risk, airspeed, roll_deg, pitch_deg, climb_filt, saturation, action)

  if action ~= ACTION.ALLOW and guard_enabled then
    local safe_pwm = theta_to_pwm(allowed_theta, P.PWM_FW:get(), P.PWM_Q:get())
    local guard_ms = math.max(100, math.floor(P.GUARD_MS:get() + 0.5))
    SRV_Channels:set_output_pwm_chan_timeout(fold_chan, safe_pwm, guard_ms)
    if action == ACTION.ABORT_TO_Q and vehicle ~= nil then
      pcall(function() vehicle:set_mode(17) end)
    end
    if not warned_guard or last_guard_reason ~= guard_reason then
      gcs:send_text(MAV_SEVERITY_WARNING,
        string.format("%s: guard reason=%s", SCRIPT_NAME, guard_reason))
      warned_guard = true
      last_guard_reason = guard_reason
    end
  else
    warned_guard = false
    last_guard_reason = nil
  end

  local mix_mode = effective_mix_mode()
  if mix_mode == MIX_MODE_MIRROR then
    for i = 1, 4 do
      -- Function IDs 94-97 are Scripting1-Scripting4 diagnostic outputs (mirror mode only).
      SRV_Channels:set_output_pwm(93 + i, pwm[i])
    end
  elseif mix_mode == MIX_MODE_CONTROL then
    local mix_theta = theta_est
    if action ~= ACTION.ALLOW and guard_enabled then
      mix_theta = allowed_theta
    end
    local dyn_factors = interpolate_factors(mix_theta)
    if apply_dynamic_motor_mix(dyn_factors) then
      motors_dynamic_active = true
    else
      motors_dynamic_active = false
      warn_missing_motors_dynamic()
      if can_takeover_motors(mix_mode, action, flight_mode, theta_target, theta_est) then
        local blend = clamp(P.MIX_BLEND:get(), 0, 1)
        for i = 1, 4 do
          local blended = math.floor(lerp(pwm[i], motor_pwm[i], blend) + 0.5)
          SRV_Channels:set_output_pwm_chan_timeout(i - 1, blended, 200)
        end
      end
    end
  else
    motors_dynamic_active = false
  end

  if math.abs(theta_target - theta_est) > 2 and (now_ms - target_changed_ms) > P.TIMEOUT:get() * 1000 then
    if not warned_timeout then
      gcs:send_text(MAV_SEVERITY_WARNING, SCRIPT_NAME .. ": fold estimate timeout")
      warned_timeout = true
    end
  end

  local loop_ms = 100
  if effective_mix_mode() == MIX_MODE_CONTROL and motors_dynamic_active then
    loop_ms = 50
  end
  return update, loop_ms
end

gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. ": loaded")
return update()
