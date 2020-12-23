#ifndef CC_HELPER_FS_H
#define CC_HELPER_FS_H

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <string>
#include <dirent.h>
#include <errno.h>

std::string getHomeDir();
bool dir_exists(std::string &path);

#endif

