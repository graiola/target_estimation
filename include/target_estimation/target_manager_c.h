/*
 * Copyright (C) 2022 Gennaro Raiola
 * Author: Gennaro Raiola
 * email:  gennaro.raiola@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

typedef void target_manager_c;

//header for C wrapper for TargetManager

#ifdef __cplusplus
extern "C" {
#endif
//TargetManager
target_manager_c * target_manager_new(const char* file);
void target_manager_init(const target_manager_c *self, const unsigned int id, const double dt0, double p0[], const double t0);
void target_manager_update_meas(const target_manager_c *self, const unsigned int id, const double dt, double meas[]);
void target_manager_update(const target_manager_c *self, const unsigned int id, const double dt);
bool target_manager_get_est_pose(const target_manager_c *self, const unsigned int id, double pose[]);
bool target_manager_get_est_twist(const target_manager_c *self, const unsigned int id, double twist[]);
bool target_manager_get_est_acceleration(const target_manager_c *self, const unsigned int id, double acceleration[]);
int  target_manager_get_n_measurements(const target_manager_c *self, const unsigned int id);
void target_manager_log(const target_manager_c *self);
void target_manager_delete(target_manager_c *self);

#ifdef __cplusplus
}
#endif
