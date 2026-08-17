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
  Transwing quadplane simulator class
*/

#pragma once

#include "SIM_Plane.h"

namespace SITL {

/*
  QuadPlane/Tiltrotor SITL model with fold-angle-dependent motor geometry.
 */
class TranswingQuadPlane : public Plane {
public:
    TranswingQuadPlane(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW TranswingQuadPlane(frame_str);
    }

private:
    Vector3f moment_of_inertia;

    float fold_theta_deg(const struct sitl_input &input) const;
    float paired_servo_angle(const struct sitl_input &input,
                             uint8_t first_idx,
                             uint8_t second_idx);
    void calculate_transwing_aero_forces(const struct sitl_input &input,
                                         Vector3f &rot_accel,
                                         Vector3f &body_accel);
    void calculate_transwing_forces(const struct sitl_input &input,
                                    Vector3f &rot_accel,
                                    Vector3f &body_accel);
};

} // namespace SITL
