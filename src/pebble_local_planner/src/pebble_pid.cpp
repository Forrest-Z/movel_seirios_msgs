#include "pebble_local_planner/pebble_pid.h"
#include <algorithm>
#include <cmath>
#include <iostream>

PID2D::PID2D()
{
}

PID2D::PID2D(double kpl, double kil, double kdl, double kpa, double kia, double kda)
{
  kp_lin_ = kpl;
  ki_lin_ = kil;
  kd_lin_ = kdl;

  kp_ang_ = kpa;
  ki_ang_ = kia;
  kd_ang_ = kda;
}

void PID2D::reset()
{
  e_int_ang_ = 0.;
  e_int_lin_ = 0.;
  prev_e_ang_ = 0.;
  prev_e_lin_ = 0.;
}

void PID2D::update(double xref, double yref, double thref,
                   double xhat, double yhat, double thhat, double dt,
                   double &vx, double &wz)
{
  // linear
  double ex = xref - xhat;
  double dex = 0.;
  if (dt > 1.e-3)
    dex = (ex - prev_e_lin_) / dt;

  double iex = e_int_lin_ + ex*dt;
  vx = kp_lin_ * ex + ki_lin_ * iex + kd_lin_ * dex;
  // if (ex < 0)
  //   vx = -max_vx_;
  // else
  //   vx = max_vx_;
  // std::cout << "ex: " << ex << ", vx raw: " << vx << std::endl;

  // angular
  // if far away, error is in heading
  double ey = yref - yhat;
  double eth = atan2(ey, ex);

  if (vx < 0 && allow_reverse_)
  {
    eth = fmod(eth + M_PI, 2.*M_PI);
    if (eth > M_PI)
      eth = -1. * (2.*M_PI - eth);
    else if (eth < -1.*M_PI)
    {
      eth = 2.*M_PI + eth;
    }
  }
  // if close by, error is to final orientation
  if (fabs(ex) < xy_tolerance_)
  {
    eth = thref - thhat;
    // std::cout << "angle ref is waypoint heading" << std::endl;
  }
  double deth = 0.;
  if (dt > 1.e-3)
    deth = (eth - prev_e_ang_) / dt;

  double ieth = e_int_ang_ + eth*dt;
  wz = kp_ang_*eth + ki_ang_*ieth + kd_ang_*deth;
  // std::cout << "eth: " << eth << ", wz raw: " << wz << std::endl;
  
  // prioritisation
  // if close enough to goal, short to zero
  // if (fabs(ex) < xy_tolerance_ && fabs(eth) < th_tolerance_)
  // {
  //   vx = 0;
  //   wz = 0;
  //   std::cout << "tolerance short circuit" << std::endl;
  // }

  // if angle error to heading is large, turn in place
  if (fabs(eth) > th_turn_)
    vx = 0.;

  // clamping
  clampVeloes(vx, wz);
  // std::cout << "final vx " << vx << " final wz " << wz << std::endl;

  // bookkeeping
  prev_e_lin_ = ex;
  prev_e_ang_ = eth;
  e_int_lin_ = iex;
  e_int_ang_ = ieth;
}

void PID2D::setLinGains(double kp, double ki, double kd)
{
  kp_lin_ = kp;
  ki_lin_ = ki;
  kd_lin_ = kd;
}

void PID2D::setAngGains(double kp, double ki, double kd)
{
  kp_ang_ = kp;
  ki_ang_ = ki;
  kd_ang_ = kd;
}

void PID2D::setMaxVeloes(double vx, double wz)
{
  max_vx_ = vx;
  max_wz_ = wz;
}

void PID2D::setTolerances(double xy_tol, double th_tol)
{
  xy_tolerance_ = xy_tol;
  th_tolerance_ = th_tol;
}

void PID2D::setTurnThresh(double th_turn)
{
  th_turn_ = th_turn;
}

void PID2D::setAllowReverse(bool allow)
{
  allow_reverse_ = allow;
}

void PID2D::clampVeloes(double &vx, double &wz)
{
  vx = std::max(-1*max_vx_, vx);
  vx = std::min(max_vx_, vx);

  wz = std::max(-1*max_wz_, wz);
  wz = std::min(max_wz_, wz);
}