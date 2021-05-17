#include <movel_hasp_vendor/license.h>

MovelLicense::MovelLicense(int featureID) : featureID_(featureID)
{
}

bool MovelLicense::login()
{
 
  static unsigned char vendorCode[] = "F2WtcTK3aODQqVwRnaeegOWFhrfANlrTkvuXASj2e6+NrYXddw4A/FBKMi30A8OIjRO/"
                                      "Wb3MLorXnc2lplrtirGVXjY8Ow/"
			              "3bbk2JwIL5YloPuSH6wgXvUSvuwAOfm6UIi5kfaPALiIOOSaoD1az69An/"
				      "to6E3eYIEKyLR2QbsCQ6fRSHTYgm1u4ReYqg/"
				      "fB79vN84POj6Zjra4bMJnPepfsTIhuLhB+"
				      "cGL98zWujVgf88YUvG3vyF6tL0Z75P25cXhjBHxZxzgzaP6OtAz7f3xmDHzvgwFt8qMKBBSPmoaZk0im"
				      "FYEBy+"
				      "nyg7RkPt2tCZfwNkd8gZsqJaUkUYXUHoQbgRHLsf0RQdYD8TJX5beNdT2ryjDW/RBHzG6Nq/"
				      "uH86TGfREwR92/"
				      "Zi1wunlEoDjc+ByKO9EiBjJnI9B20apnan7HFP2uSmbBviNQpKBpKT92fTr/"
				      "n49BoliEswUh0uwu++UpIWb6fWkQgzcAQEFBqtp/nLd5ojL9ZafLGDoWln+y3jBuWxfS/"
				      "9f4a3z8ynQw9pFF7YWMHItmmG9yLBIfvIdL4pDkvpOS5PpTOCuITLB0npjY3YYmpXpn+"
				      "1nQ9NeY1XN4wmcdBhHNE4UERwyj7vTtZTJKd09Q0ocADuybigcbSLW8CTQ0qXJzGBvQXir5WSP+"
				      "llFmuPkqVEvdtexDeB9c6X0SAhX69yLBC06PCHwrog1XzPwHo4KQcWyt8K06c5ImLQsxrm86FnCtvEBJ"
			              "73WpVh8"
                                      "sLZWenUxi284FC6NqcqMKvXqK3b6jy2o7LCpTMw2FN6x/oTCKNh4vhYNMTIWWX0o4q/"
                                      "l09tf27IH0JwHaklwFu0QQwAw+"
                                      "XETl7aNWCS310IF6Rz6xmhOIIk8quF0DviaerqkY5RJjTQ2JmhGFKXjLZG685LG5uNoGHxFj95/"
                                      "Zq9yLJ8nWxYZ5J73NSaCbocyEeGw+tlJVE4oiKBcTMX7+WGYzpoLQPxzR6A==";				      

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
