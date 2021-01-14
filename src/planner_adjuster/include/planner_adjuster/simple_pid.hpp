#ifndef SIMPLE_PID_HPP
#define SIMPLE_PID_HPP

class SimplePID 
{
public:
  SimplePID();
  SimplePID(double k_p, double k_i, double k_d);
  ~SimplePID(){}

  void reset();
  void setRef(double x_ref);
  void setGains(double k_p, double k_i, double k_d);
  double getRef();

  double update(double x, double dt);

private:
  double k_p_, k_i_, k_d_;
  double e_prev_, e_int_;
  double x_ref_;
};

#endif