#include <movel_hasp_vendor/hasp_api_cpp.h>
#include <movel_hasp_vendor/errorprinter.h>

class MovelLicense
{
private:
  int featureID_;
  haspStatus status_;
  Chasp hasp_;

public:
  MovelLicense();

  bool login();
  void logout();
};
