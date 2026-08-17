/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Transwing quadplane simulator class.

  This model keeps ArduPlane's native QuadPlane/Tiltrotor control path, but
  replaces the fixed SITL multicopter frame with a geometry table where motor
  position and thrust direction change with the fold angle.
*/

#include "SIM_TranswingQuadPlane.h"

#include <stdio.h>

using namespace SITL;

namespace {

constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t MOTOR_SERVO_OFFSET = 0;  // SERVO1..SERVO4 are Motor1..Motor4
constexpr uint8_t FOLD_SERVO_IDX = 10;     // SERVO11_FUNCTION=41
constexpr uint8_t AILERON_SERVO_A_IDX = 4;   // SERVO5_FUNCTION=4
constexpr uint8_t AILERON_SERVO_B_IDX = 5;   // SERVO6_FUNCTION=4
constexpr uint8_t ELEVATOR_SERVO_A_IDX = 6;  // SERVO7_FUNCTION=19
constexpr uint8_t ELEVATOR_SERVO_B_IDX = 7;  // SERVO8_FUNCTION=19
constexpr uint8_t RUDDER_SERVO_A_IDX = 8;    // SERVO9_FUNCTION=21
constexpr uint8_t RUDDER_SERVO_B_IDX = 9;    // SERVO10_FUNCTION=21
constexpr float PWM_FOLD_Q = 1000.0f;      // native tilt value 0.0, hover/up
constexpr float PWM_FOLD_FW = 2000.0f;     // native tilt value 1.0, forward
constexpr float HOVER_THROTTLE = 0.50f;
constexpr float MAX_THRUST_WEIGHT_RATIO = 2.0f;
constexpr float ROTOR_TORQUE_COEFF = 0.035f;
constexpr float TERMINAL_ROTATION_RATE = radians(720.0f);
constexpr float MOT_PWM_MIN = 1000.0f;
constexpr float MOT_PWM_MAX = 2000.0f;
constexpr float MOT_SPIN_MIN = 0.12f;
constexpr float MOT_SPIN_MAX = 0.95f;

struct MotorGeometry {
    Vector3f position;
    Vector3f thrust_vector;
    float yaw_drag_sign;
};

struct FoldGeometry {
    float theta_deg;
    MotorGeometry motor[MOTOR_COUNT];
};

const FoldGeometry geometry_table[] {
    {
        0.0f,
        {
            { Vector3f{ 0.687500f,  0.320000f, -0.045000f}, Vector3f{1.000000f, 0.0f,  0.000000f},  1.0f },
            { Vector3f{ 0.682500f, -0.980000f, -0.060000f}, Vector3f{1.000000f, 0.0f,  0.000000f},  1.0f },
            { Vector3f{ 0.687500f, -0.320000f, -0.045000f}, Vector3f{1.000000f, 0.0f,  0.000000f}, -1.0f },
            { Vector3f{ 0.682500f,  0.980000f, -0.060000f}, Vector3f{1.000000f, 0.0f,  0.000000f}, -1.0f },
        }
    },
    {
        15.0f,
        {
            { Vector3f{ 0.660093f,  0.332593f, -0.075741f}, Vector3f{0.996815f, 0.0f, -0.079745f},  1.0f },
            { Vector3f{ 0.608426f, -0.941481f, -0.081852f}, Vector3f{0.996815f, 0.0f, -0.079745f},  1.0f },
            { Vector3f{ 0.660093f, -0.332593f, -0.075741f}, Vector3f{0.996815f, 0.0f, -0.079745f}, -1.0f },
            { Vector3f{ 0.608426f,  0.941481f, -0.081852f}, Vector3f{0.996815f, 0.0f, -0.079745f}, -1.0f },
        }
    },
    {
        30.0f,
        {
            { Vector3f{ 0.591574f,  0.364074f, -0.152593f}, Vector3f{0.943858f, 0.0f, -0.330350f},  1.0f },
            { Vector3f{ 0.423241f, -0.845185f, -0.136481f}, Vector3f{0.943858f, 0.0f, -0.330350f},  1.0f },
            { Vector3f{ 0.591574f, -0.364074f, -0.152593f}, Vector3f{0.943858f, 0.0f, -0.330350f}, -1.0f },
            { Vector3f{ 0.423241f,  0.845185f, -0.136481f}, Vector3f{0.943858f, 0.0f, -0.330350f}, -1.0f },
        }
    },
    {
        45.0f,
        {
            { Vector3f{ 0.502500f,  0.405000f, -0.252500f}, Vector3f{0.707107f, 0.0f, -0.707107f},  1.0f },
            { Vector3f{ 0.182500f, -0.720000f, -0.207500f}, Vector3f{0.707107f, 0.0f, -0.707107f},  1.0f },
            { Vector3f{ 0.502500f, -0.405000f, -0.252500f}, Vector3f{0.707107f, 0.0f, -0.707107f}, -1.0f },
            { Vector3f{ 0.182500f,  0.720000f, -0.207500f}, Vector3f{0.707107f, 0.0f, -0.707107f}, -1.0f },
        }
    },
    {
        60.0f,
        {
            { Vector3f{ 0.413426f,  0.445926f, -0.352407f}, Vector3f{0.330350f, 0.0f, -0.943858f},  1.0f },
            { Vector3f{-0.058241f, -0.594815f, -0.278519f}, Vector3f{0.330350f, 0.0f, -0.943858f},  1.0f },
            { Vector3f{ 0.413426f, -0.445926f, -0.352407f}, Vector3f{0.330350f, 0.0f, -0.943858f}, -1.0f },
            { Vector3f{-0.058241f,  0.594815f, -0.278519f}, Vector3f{0.330350f, 0.0f, -0.943858f}, -1.0f },
        }
    },
    {
        75.0f,
        {
            { Vector3f{ 0.344907f,  0.477407f, -0.429259f}, Vector3f{0.079745f, 0.0f, -0.996815f},  1.0f },
            { Vector3f{-0.243426f, -0.498519f, -0.333148f}, Vector3f{0.079745f, 0.0f, -0.996815f},  1.0f },
            { Vector3f{ 0.344907f, -0.477407f, -0.429259f}, Vector3f{0.079745f, 0.0f, -0.996815f}, -1.0f },
            { Vector3f{-0.243426f,  0.498519f, -0.333148f}, Vector3f{0.079745f, 0.0f, -0.996815f}, -1.0f },
        }
    },
    {
        90.0f,
        {
            { Vector3f{ 0.317500f,  0.490000f, -0.460000f}, Vector3f{0.000000f, 0.0f, -1.000000f},  1.0f },
            { Vector3f{-0.317500f, -0.460000f, -0.355000f}, Vector3f{0.000000f, 0.0f, -1.000000f},  1.0f },
            { Vector3f{ 0.317500f, -0.490000f, -0.460000f}, Vector3f{0.000000f, 0.0f, -1.000000f}, -1.0f },
            { Vector3f{-0.317500f,  0.460000f, -0.355000f}, Vector3f{0.000000f, 0.0f, -1.000000f}, -1.0f },
        }
    },
};

float pwm_to_motor_command(uint16_t pwm)
{
    if (pwm == 0) {
        return 0.0f;
    }
    const float pwm_range = MOT_PWM_MAX - MOT_PWM_MIN;
    const float pwm_thrust_min = MOT_PWM_MIN + MOT_SPIN_MIN * pwm_range;
    const float pwm_thrust_max = MOT_PWM_MIN + MOT_SPIN_MAX * pwm_range;
    const float pwm_thrust_range = pwm_thrust_max - pwm_thrust_min;

    return constrain_float((float(pwm) - pwm_thrust_min) / pwm_thrust_range, 0.0f, 1.0f);
}

MotorGeometry interpolate_geometry(float theta_deg, uint8_t motor_idx)
{
    if (theta_deg <= geometry_table[0].theta_deg) {
        return geometry_table[0].motor[motor_idx];
    }

    const uint8_t last = ARRAY_SIZE(geometry_table) - 1;
    if (theta_deg >= geometry_table[last].theta_deg) {
        return geometry_table[last].motor[motor_idx];
    }

    for (uint8_t i = 0; i < last; i++) {
        const auto &a = geometry_table[i];
        const auto &b = geometry_table[i + 1];
        if (theta_deg > b.theta_deg) {
            continue;
        }
        const float t = (theta_deg - a.theta_deg) / (b.theta_deg - a.theta_deg);
        MotorGeometry out {
            a.motor[motor_idx].position + (b.motor[motor_idx].position - a.motor[motor_idx].position) * t,
            a.motor[motor_idx].thrust_vector + (b.motor[motor_idx].thrust_vector - a.motor[motor_idx].thrust_vector) * t,
            a.motor[motor_idx].yaw_drag_sign,
        };
        out.thrust_vector.normalize();
        return out;
    }

    return geometry_table[last].motor[motor_idx];
}

} // namespace

TranswingQuadPlane::TranswingQuadPlane(const char *frame_str) :
    Plane(frame_str)
{
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    thrust_scale = 0.0f;
    mass = 3.6f;
    moment_of_inertia = Vector3f{0.45f, 0.65f, 0.90f};
    frame_height = 0.45f;
    lock_step_scheduled = true;

    filtered_servo_setup(FOLD_SERVO_IDX, PWM_FOLD_Q, PWM_FOLD_FW, 90.0f);
    printf("Loaded Transwing QuadPlane SITL dynamic geometry\n");
}

float TranswingQuadPlane::fold_theta_deg(const struct sitl_input &input) const
{
    const uint16_t pwm = input.servos[FOLD_SERVO_IDX] == 0 ? PWM_FOLD_Q : input.servos[FOLD_SERVO_IDX];
    const float tilt_forward = constrain_float((float(pwm) - PWM_FOLD_Q) / (PWM_FOLD_FW - PWM_FOLD_Q), 0.0f, 1.0f);
    return 90.0f * (1.0f - tilt_forward);
}

float TranswingQuadPlane::paired_servo_angle(const struct sitl_input &input,
                                             uint8_t first_idx,
                                             uint8_t second_idx)
{
    return 0.5f * (filtered_servo_angle(input, first_idx) + filtered_servo_angle(input, second_idx));
}

void TranswingQuadPlane::calculate_transwing_aero_forces(const struct sitl_input &input,
                                                         Vector3f &rot_accel,
                                                         Vector3f &body_accel)
{
    const float aileron = paired_servo_angle(input, AILERON_SERVO_A_IDX, AILERON_SERVO_B_IDX);
    const float elevator = paired_servo_angle(input, ELEVATOR_SERVO_A_IDX, ELEVATOR_SERVO_B_IDX);
    const float rudder = paired_servo_angle(input, RUDDER_SERVO_A_IDX, RUDDER_SERVO_B_IDX);
    constexpr float fixed_wing_thrust = 0.0f;

    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);

    const Vector3f force = getForce(aileron, elevator, rudder);
    rot_accel = getTorque(aileron, elevator, rudder, fixed_wing_thrust, force);
    body_accel = force / mass;
}

void TranswingQuadPlane::calculate_transwing_forces(const struct sitl_input &input,
                                                    Vector3f &rot_accel,
                                                    Vector3f &body_accel)
{
    const float theta = fold_theta_deg(input);
    const float max_thrust_per_motor = (mass * GRAVITY_MSS * MAX_THRUST_WEIGHT_RATIO) / MOTOR_COUNT;
    Vector3f total_force;
    Vector3f total_torque;
    float throttle_sum = 0.0f;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const uint8_t servo_idx = MOTOR_SERVO_OFFSET + i;
        const float command = pwm_to_motor_command(input.servos[servo_idx]);
        const float thrust_n = command * max_thrust_per_motor;
        const MotorGeometry geom = interpolate_geometry(theta, i);

        const Vector3f force = geom.thrust_vector * thrust_n;
        const Vector3f arm_torque = geom.position % force;
        const Vector3f rotor_torque = geom.thrust_vector * (-geom.yaw_drag_sign * thrust_n * ROTOR_TORQUE_COEFF);

        total_force += force;
        total_torque += arm_torque + rotor_torque;
        throttle_sum += command;

        motor_mask |= 1U << servo_idx;
        rpm[servo_idx] = command * 7000.0f;
    }

    rot_accel.x = total_torque.x / moment_of_inertia.x;
    rot_accel.y = total_torque.y / moment_of_inertia.y;
    rot_accel.z = total_torque.z / moment_of_inertia.z;

    rot_accel.x -= gyro.x * radians(400.0f) / TERMINAL_ROTATION_RATE;
    rot_accel.y -= gyro.y * radians(400.0f) / TERMINAL_ROTATION_RATE;
    rot_accel.z -= gyro.z * radians(400.0f) / TERMINAL_ROTATION_RATE;

    body_accel = total_force / mass;

    const float throttle_avg = throttle_sum / MOTOR_COUNT;
    battery_voltage = sitl->batt_voltage - 0.8f * throttle_avg;
    battery_current = 65.0f * sq(throttle_avg);
    battery.set_current(battery_current);

    add_noise(throttle_avg / HOVER_THROTTLE);
}

void TranswingQuadPlane::update(const struct sitl_input &input)
{
    update_wind(input);

    Vector3f rot_accel;
    Vector3f aero_accel_body;
    calculate_transwing_aero_forces(input, rot_accel, aero_accel_body);

    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;
    calculate_transwing_forces(input, quad_rot_accel, quad_accel_body);

    rot_accel += quad_rot_accel;
    accel_body = aero_accel_body + quad_accel_body;

    update_dynamics(rot_accel);
    update_external_payload(input);
    update_position();
    time_advance();
    update_mag_field_bf();
}
