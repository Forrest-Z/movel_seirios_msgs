#include <anti_shin_buster/pitch_estimator.hpp>
#include <movel_hasp_vendor/license.h>

int main(int argc, char** argv)
{
#ifdef MOVEL_LICENSE
  MovelLicense ml(10);
  if (!ml.login())
    return 1;
#endif

  ros::init(argc, argv, "pitch_estimator");
  PitchEstimator PE;

#ifdef MOVEL_LICENSE
  ml.logout();
#endif

  return 0;
}
