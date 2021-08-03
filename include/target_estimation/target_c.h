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
bool target_manager_get_int_pose(const target_manager_c *self, const unsigned int id, const double t, const double pos_th, const double ang_th, double int_pose[]);
bool target_manager_get_est_pose(const target_manager_c *self, const unsigned int id, double pose[]);
bool target_manager_get_est_twist(const target_manager_c *self, const unsigned int id, double twist[]);
bool target_manager_get_est_acceleration(const target_manager_c *self, const unsigned int id, double acceleration[]);
void target_manager_set_int_sphere(const target_manager_c *self, double origin[], double radius);
int  target_manager_get_n_measurements(const target_manager_c *self, const unsigned int id);
void target_manager_log(const target_manager_c *self);
void target_manager_delete(target_manager_c *self);

#ifdef __cplusplus
}
#endif
