#include <helpers/fs_helper.hpp>

std::string getHomeDir()
{
  const char *homedir;

  if ((homedir = getenv("HOME")) == NULL)
    homedir = getpwuid(getuid())->pw_dir;
  return std::string(homedir);
}

// Awaiting C++17  to use std::filesystem
bool dir_exists(std::string &path)
{
  DIR* dir = opendir(path.c_str());
  if (dir)
  {
    closedir(dir);
    return true;
  }
  else
    return false;
}
