-- Transwing fold-angle estimator, dynamic mix prototype, and transition guard.
-- SITL-first script: motor mix outputs stay isolated unless TW_LOG_ONLY=0.

local SCRIPT_NAME = "TW-DYNMIX"
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_WARNING = 4

local TABLE_KEY = 91
assert(param:add_table(TABLE_KEY, "TW_", 20), "could not add TW_ parameter table")

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
assert(param:add_param(TABLE_KEY, 15, "SAFE_MIN", 60), "could not add TW_SAFE_MIN")
assert(param:add_param(TABLE_KEY, 16, "BLEND_AS", 18), "could not add TW_BLEND_AS")
assert(param:add_param(TABLE_KEY, 17, "FW_AS", 25), "could not add TW_FW_AS")
assert(param:add_param(TABLE_KEY, 18, "ATT_DANG", 35), "could not add TW_ATT_DANG")
assert(param:add_param(TABLE_KEY, 19, "DESC_DANG", -5), "could not add TW_DESC_DANG")
assert(param:add_param(TABLE_KEY, 20, "SAT_PWM", 1900), "could not add TW_SAT_PWM")

local P = {}
for _, name in ipairs({
  "ENABLE", "LOG_ONLY", "FOLD_CH", "PWM_FW", "PWM_Q", "RATE_UP", "RATE_DN",
  "TIMEOUT", "OUT_GAIN", "THR", "ROLL", "PITCH", "YAW", "GUARD", "SAFE_MIN",
  "BLEND_AS", "FW_AS", "ATT_DANG", "DESC_DANG", "SAT_PWM"
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
local ACTION = { ALLOW = 0, HOLD_THETA = 1 }

local theta_est = 0
local theta_target = 0
local last_ms = millis():tofloat()
local target_changed_ms = last_ms
local last_target = nil
local warned_timeout = false
local warned_guard = false
local announced = false

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
  if theta >= 75 then return PHASE.HOVER end
  if theta >= 60 then return PHASE.ACCEL end
  if theta >= 45 then return PHASE.BLEND end
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
  for i = 1, #pwm do
    if pwm[i] >= sat_pwm or pwm[i] <= 1100 then return 1 end
  end
  return 0
end

local function evaluate_transition(theta, target, airspeed, roll_deg, pitch_deg, climb_rate, saturation)
  local phase = phase_for_theta(theta)
  local risk = RISK.OK
  local action = ACTION.ALLOW
  local allowed = target
  local safe_min = P.SAFE_MIN:get()

  if theta < safe_min and airspeed < P.BLEND_AS:get() then risk = RISK.DANGER end
  if theta < 45 and airspeed < P.FW_AS:get() then risk = RISK.DANGER end
  if math.abs(roll_deg) > P.ATT_DANG:get() or math.abs(pitch_deg) > P.ATT_DANG:get() then risk = RISK.DANGER end
  if climb_rate < P.DESC_DANG:get() then risk = RISK.DANGER end
  if saturation > 0.5 then risk = RISK.DANGER end

  if risk == RISK.DANGER and target < safe_min then
    action = ACTION.HOLD_THETA
    allowed = safe_min
  end

  return phase, risk, action, allowed
end

local function update()
  local now_ms = millis():tofloat()
  local dt = (now_ms - last_ms) * 0.001
  last_ms = now_ms

  if P.ENABLE:get() < 0.5 then return update, 100 end

  if not announced then
    gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. ": running, log-only=" .. tostring(P.LOG_ONLY:get()))
    announced = true
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
  local throttle = clamp(P.THR:get(), 0, 1)
  local roll = clamp(P.ROLL:get(), -1, 1)
  local pitch = clamp(P.PITCH:get(), -1, 1)
  local yaw = clamp(P.YAW:get(), -1, 1)
  local gain = clamp(P.OUT_GAIN:get(), 0, 1)

  local pwm = {}
  for i = 1, 4 do pwm[i] = mix_to_pwm(factors[i], throttle, roll, pitch, yaw, gain) end

  local airspeed = read_airspeed()
  local roll_deg, pitch_deg = read_attitude_deg()
  local climb_rate = read_climb_rate()
  local saturation = is_saturated(pwm, P.SAT_PWM:get())
  local phase, risk, action, allowed_theta = evaluate_transition(
    theta_est, theta_target, airspeed, roll_deg, pitch_deg, climb_rate, saturation
  )

  logger:write("TWNG", "Targ,Est,Fold,M1,M2,M3,M4", "fffffff",
    theta_target, theta_est, fold_pwm or 0, pwm[1], pwm[2], pwm[3], pwm[4])

  logger:write("TWTR", "Targ,Est,Allow,Phase,Risk,AS,Roll,Pitch,Climb,Sat,Act", "fffffffffff",
    theta_target, theta_est, allowed_theta, phase, risk, airspeed, roll_deg, pitch_deg, climb_rate, saturation, action)

  if action == ACTION.HOLD_THETA and P.GUARD:get() > 0.5 then
    local safe_pwm = theta_to_pwm(allowed_theta, P.PWM_FW:get(), P.PWM_Q:get())
    SRV_Channels:set_output_pwm_chan_timeout(fold_chan, safe_pwm, 300)
    if not warned_guard then
      gcs:send_text(MAV_SEVERITY_WARNING, SCRIPT_NAME .. ": transition guard holding fold angle")
      warned_guard = true
    end
  else
    warned_guard = false
  end

  if P.LOG_ONLY:get() < 0.5 then
    for i = 1, 4 do
      -- Function IDs 94-97 are Scripting1-Scripting4 outputs.
      -- Map isolated SITL outputs to SERVOx_FUNCTION 94-97 before enabling TW_LOG_ONLY=0.
      SRV_Channels:set_output_pwm(93 + i, pwm[i])
    end
  end

  if math.abs(theta_target - theta_est) > 2 and (now_ms - target_changed_ms) > P.TIMEOUT:get() * 1000 then
    if not warned_timeout then
      gcs:send_text(MAV_SEVERITY_WARNING, SCRIPT_NAME .. ": fold estimate timeout")
      warned_timeout = true
    end
  end

  return update, 100
end

gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. ": loaded")
return update()
