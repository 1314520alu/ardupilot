-- Transwing open-loop fold-angle estimator and dynamic mix prototype.
-- SITL-first script: default mode logs only and does not override real motor outputs.

local SCRIPT_NAME = "TW-DYNMIX"
local MAV_SEVERITY_INFO = 6
local MAV_SEVERITY_WARNING = 4

local TABLE_KEY = 91
assert(param:add_table(TABLE_KEY, "TW_", 13), "could not add TW_ parameter table")

assert(param:add_param(TABLE_KEY, 1, "ENABLE", 1), "could not add TW_ENABLE")
assert(param:add_param(TABLE_KEY, 2, "LOG_ONLY", 1), "could not add TW_LOG_ONLY")
assert(param:add_param(TABLE_KEY, 3, "FOLD_CH", 10), "could not add TW_FOLD_CH")
assert(param:add_param(TABLE_KEY, 4, "PWM_FW", 1100), "could not add TW_PWM_FW")
assert(param:add_param(TABLE_KEY, 5, "PWM_Q", 1900), "could not add TW_PWM_Q")
assert(param:add_param(TABLE_KEY, 6, "RATE_UP", 4.5), "could not add TW_RATE_UP")
assert(param:add_param(TABLE_KEY, 7, "RATE_DN", 4.5), "could not add TW_RATE_DN")
assert(param:add_param(TABLE_KEY, 8, "TIMEOUT", 25), "could not add TW_TIMEOUT")
assert(param:add_param(TABLE_KEY, 9, "OUT_GAIN", 0.2), "could not add TW_OUT_GAIN")
assert(param:add_param(TABLE_KEY, 10, "THR", 0), "could not add TW_THR")
assert(param:add_param(TABLE_KEY, 11, "ROLL", 0), "could not add TW_ROLL")
assert(param:add_param(TABLE_KEY, 12, "PITCH", 0), "could not add TW_PITCH")
assert(param:add_param(TABLE_KEY, 13, "YAW", 0), "could not add TW_YAW")

local P = {}
for _, name in ipairs({
  "ENABLE", "LOG_ONLY", "FOLD_CH", "PWM_FW", "PWM_Q", "RATE_UP", "RATE_DN",
  "TIMEOUT", "OUT_GAIN", "THR", "ROLL", "PITCH", "YAW"
}) do
  P[name] = Parameter("TW_" .. name)
end

local FACTOR_TABLE = {
  {
    theta = 0,
    motors = {
      { name = "M1", throttle = 0, roll = 0, pitch = -0.750000, yaw_force = -0.326531, yaw_drag = 1 },
      { name = "M2", throttle = 0, roll = 0, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 0, roll = 0, pitch = -0.750000, yaw_force = 0.326531, yaw_drag = -1 },
      { name = "M4", throttle = 0, roll = 0, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 15,
    motors = {
      { name = "M1", throttle = 1, roll = -0.353265, pitch = -0.778152, yaw_force = -0.353265, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.353265, pitch = -0.778152, yaw_force = 0.353265, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 30,
    motors = {
      { name = "M1", throttle = 1, roll = -0.430762, pitch = -0.053440, yaw_force = -0.430762, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.430762, pitch = -0.053440, yaw_force = 0.430762, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 45,
    motors = {
      { name = "M1", throttle = 1, roll = -0.562500, pitch = 0.466667, yaw_force = -0.562500, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.562500, pitch = 0.466667, yaw_force = 0.562500, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 60,
    motors = {
      { name = "M1", throttle = 1, roll = -0.749689, pitch = 0.400925, yaw_force = -0.749689, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.749689, pitch = 0.400925, yaw_force = 0.749689, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 75,
    motors = {
      { name = "M1", throttle = 1, roll = -0.957652, pitch = 0.342289, yaw_force = -0.957652, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 1.000000, pitch = -1.000000, yaw_force = 1.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 0.957652, pitch = 0.342289, yaw_force = 0.957652, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -1.000000, pitch = -1.000000, yaw_force = -1.000000, yaw_drag = -1 },
    },
  },
  {
    theta = 90,
    motors = {
      { name = "M1", throttle = 1, roll = -1.000000, pitch = 0.322917, yaw_force = 0.000000, yaw_drag = 1 },
      { name = "M2", throttle = 1, roll = 0.938776, pitch = -1.000000, yaw_force = 0.000000, yaw_drag = 1 },
      { name = "M3", throttle = 1, roll = 1.000000, pitch = 0.322917, yaw_force = 0.000000, yaw_drag = -1 },
      { name = "M4", throttle = 1, roll = -0.938776, pitch = -1.000000, yaw_force = 0.000000, yaw_drag = -1 },
    },
  },
}

local theta_est = 0
local theta_target = 0
local last_ms = millis():tofloat()
local target_changed_ms = last_ms
local last_target = nil
local warned_timeout = false
local announced = false

local function clamp(value, min_value, max_value)
  if value < min_value then
    return min_value
  end
  if value > max_value then
    return max_value
  end
  return value
end

local function lerp(a, b, ratio)
  return a + (b - a) * ratio
end

local function pwm_to_theta_target(pwm, pwm_fw, pwm_q)
  if pwm == nil then
    return theta_target
  end
  if pwm_fw == pwm_q then
    return theta_target
  end

  local ratio = clamp((pwm - pwm_fw) / (pwm_q - pwm_fw), 0, 1)
  return ratio * 90
end

local function step_theta_estimate(theta, target, dt, rate_up, rate_dn)
  local delta = target - theta
  if delta == 0 or dt <= 0 then
    return theta
  end

  local rate = rate_dn
  if delta > 0 then
    rate = rate_up
  end

  local max_step = math.abs(rate * dt)
  if math.abs(delta) <= max_step then
    return target
  end

  if delta > 0 then
    return theta + max_step
  end
  return theta - max_step
end

local function interpolate_factors(theta)
  if theta <= FACTOR_TABLE[1].theta then
    return FACTOR_TABLE[1].motors
  end

  if theta >= FACTOR_TABLE[#FACTOR_TABLE].theta then
    return FACTOR_TABLE[#FACTOR_TABLE].motors
  end

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
    gain * (
      roll * factor.roll +
      pitch * factor.pitch +
      yaw * (factor.yaw_force + factor.yaw_drag)
    )
  return math.floor(clamp(hover_pwm + normalized * (max_pwm - hover_pwm), min_pwm, max_pwm) + 0.5)
end

local function update()
  local now_ms = millis():tofloat()
  local dt = (now_ms - last_ms) * 0.001
  last_ms = now_ms

  if P.ENABLE:get() < 0.5 then
    return update, 100
  end

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

  theta_est = step_theta_estimate(
    theta_est,
    theta_target,
    dt,
    P.RATE_UP:get(),
    P.RATE_DN:get()
  )

  local factors = interpolate_factors(theta_est)
  local throttle = clamp(P.THR:get(), 0, 1)
  local roll = clamp(P.ROLL:get(), -1, 1)
  local pitch = clamp(P.PITCH:get(), -1, 1)
  local yaw = clamp(P.YAW:get(), -1, 1)
  local gain = clamp(P.OUT_GAIN:get(), 0, 1)

  local pwm = {}
  for i = 1, 4 do
    pwm[i] = mix_to_pwm(factors[i], throttle, roll, pitch, yaw, gain)
  end

  logger:write(
    "TWNG",
    "Targ,Est,Fold,M1,M2,M3,M4",
    "fffffff",
    theta_target,
    theta_est,
    fold_pwm or 0,
    pwm[1],
    pwm[2],
    pwm[3],
    pwm[4]
  )

  if P.LOG_ONLY:get() < 0.5 then
    for i = 1, 4 do
      -- Function IDs 94-97 are Scripting1-Scripting4 outputs.
      -- Map isolated SITL outputs to SERVOx_FUNCTION 94-97 before enabling TW_LOG_ONLY=0.
      SRV_Channels:set_output_pwm(93 + i, pwm[i])
    end
  end

  local timeout_s = P.TIMEOUT:get()
  if math.abs(theta_target - theta_est) > 2 and (now_ms - target_changed_ms) > timeout_s * 1000 then
    if not warned_timeout then
      gcs:send_text(MAV_SEVERITY_WARNING, SCRIPT_NAME .. ": fold estimate timeout")
      warned_timeout = true
    end
  end

  return update, 100
end

gcs:send_text(MAV_SEVERITY_INFO, SCRIPT_NAME .. ": loaded")
return update()
