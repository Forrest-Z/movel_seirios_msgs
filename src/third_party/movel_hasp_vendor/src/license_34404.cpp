#include <movel_hasp_vendor/license.h>

MovelLicense::MovelLicense() : featureID_(2)
{
}

bool MovelLicense::login()
{
  static unsigned char vendorCode[] = "2ZOWJCCnIzblyUFFlYgbhlr6hI1whIAJuve0QWEbvCroCQxmPBrhzuzJLM78hiDOSKzaoFukUakeKYtUsySCPDgdTK+Bau3l7BZ0CLOBS51GXKjplBd/"
	  			      "f3b/rsQzZxBJJrVSqAgjOLnjttWA7JM6jcQTEARjL6Ak7oNsIdY0Etfx2iPTIoVBFbH54uNy4W4RGcCE294lZ26"
                                      "xJ9gZlajU2VtIKAjm/PkyKpeaRjx8IJU7IqIVB5qx6g27h4mJU2bl16art3IKVy7r0mMAp3jWUSrhBRcAyNBI9trJZUpxRbRgS0EC3dgCCZ+"
                                      "8qQPhDz7uLssyCnO0FwDcPj7R8HwUAqipwc0FcqOfisKpJAxUm46nOa2LOVVDnKYqtoDlY9QX6hudLsfgNGxCqwcsOhJakQ"
                                      "mx4X0BTBacALkMzmGbG6+2XQP2rbD37O08kmKEkF8PYzRhuN02fWSUlsXCBUNoX7EwzMKUBqA+o3t1GnvLmN12I+C8rieegtmRibGOoM/"
                                      "XlPhTIvU480/a7erbztCf8x3GmltRXPjVbi1zl5VieovvNtySjW7KpDyxLIxnOG4lV5cT8H4tAos7AjucvgIv+wXmwFFXzm7Wi"
                                      "1vjlqpgtCuV4CacqgHSIoYbxcOP1dq6ijH+AogmT1/0n04y9wHSiGDnOS70hzeXP3h10ibz5mTYXDkWTmQbQCKSJAvNYUOc0a5xTz8TTR/"
                                      "YlZfEOJRdqsdAU9avIowdmTu6lSlb61uOWwaEhFfrzcbses1/kFfuZWYd0WI6NbfjdC3oLCzSCwZDbbif7s257OAu6YOjctHA"
                                      "drpYQsLlf6M9klODCBzvFUrHGVJlEew1PKbAZcMD6mYpwok+Y40Wkp1R+ujH5ZgB7Vctw/kEOgQu4yc9KvehaP"
                                      "/qFE0OhreCb43KnmYGz6szpKYQGKH/1iwrE0yF5bwiWel8ePrPQFHUhtY3fEHx6Hz2CG1/jioY2P9Bfp5p6w==";

  // Prints the error messages for the return values of the functions
  const char* localScope = "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
                           "<haspscope>"
                           "    <license_manager hostname =\"localhost\" />"
                           "</haspscope>";
  ErrorPrinter errorPrinter;

  ChaspFeature feature = ChaspFeature::fromFeature(featureID_);
  Chasp hasp(feature);
  hasp_ = hasp;
  status_ = hasp_.login(vendorCode, localScope);
  errorPrinter.printError(status_);
  if (!HASP_SUCCEEDED(status_))
    return false;
  else
    return true;
}

void MovelLicense::logout()
{
  if (HASP_SUCCEEDED(status_))
    hasp_.logout();
}
