#ifndef LIOF_UTILS_H
#define LIOF_UTILS_H

// [0..2PI]
inline double normalize(double x)
{
  x = fmod(x, 2 * M_PI);
  if (x < 0)
    x += 2 * M_PI;
  return x;
}
#endif