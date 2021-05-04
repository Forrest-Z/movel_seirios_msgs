#include <dafan_docking/simple_pid.hpp>

SimplePID::SimplePID()
{
  setGains(0.0, 0.0, 0.0);
  reset();
}

SimplePID::SimplePID(double k_p, double k_i, double k_d)
{
  setGains(k_p, k_i, k_d);
  reset();
}

void SimplePID::reset()
{
  e_prev_ = 0.0;
  e_int_ = 0.0;
}

void SimplePID::setRef(double x_ref)
{
  x_ref_ = x_ref;
}

void SimplePID::setGains(double k_p, double k_i, double k_d)
{
  k_p_ = k_p;
  k_i_ = k_i;
  k_d_ = k_d;
}

double SimplePID::getRef()
{
  return x_ref_;
}

double SimplePID::update(double x, double dt)
{
  double u = 0.0;
  double e = x_ref_ - x;
  double e_diff = 0.0;
  if (dt > 0.0)
    e_diff = (e - e_prev_)/dt;
  else
    e_diff = 0.0;
  e_int_ += e*dt;
  
  e_prev_ = e;
  
  u += k_p_ * e + k_i_ * e_int_ + k_d_ * e_diff;
  return u;
}