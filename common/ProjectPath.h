#ifndef DEPLOY_REAL_COMMON_PROJECT_PATH_H
#define DEPLOY_REAL_COMMON_PROJECT_PATH_H

#include <filesystem>
#include <string>

#ifndef DEPLOY_REAL_SOURCE_DIR
#error "DEPLOY_REAL_SOURCE_DIR must be defined by CMake"
#endif

namespace deploy_real
{

inline std::string ProjectPath(const std::filesystem::path &relative_path)
{
    return (std::filesystem::path(DEPLOY_REAL_SOURCE_DIR) / relative_path)
        .lexically_normal()
        .string();
}

} // namespace deploy_real

#endif
