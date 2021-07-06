#ifndef pebble_pid_h
#define pebble_pid_h

class PID2D
{
public:
  PID2D();
  PID2D(double kpl, double kil, double kdl, double kpa, double kia, double kda);
  ~PID2D(){}

  void reset();
  void update(double xref, double yref, double thref,
              double xhat, double yhat, double thhat, double dt,
              double &vx, double &wz);
  void setLinGains(double kp, double ki, double kd);
  void setAngGains(double kp, double ki, double kd);
  void setMaxVeloes(double vx, double wz);
  void setTolerances(double xy_tol, double th_tol);
  void setTurnThresh(double th_turn);
  void clampVeloes(double &vx, double &wz);
  
private:
  // parameters
  double kp_lin_, ki_lin_, kd_lin_;
  double kp_ang_, ki_ang_, kd_ang_;
  double max_vx_;
  double max_wz_;
  double xy_tolerance_;
  double th_tolerance_;
  double th_turn_; // angle error to prioritise turning in place

  // bookkeeping
  double e_int_lin_;
  double e_int_ang_;
  double prev_e_lin_;
  double prev_e_ang_;
};

#endif