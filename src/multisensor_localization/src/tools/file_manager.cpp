#include "multisensor_localization/tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace multisensor_localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if (!ofs) {
        LOG(WARNING) << "Cannot create file:\n" << file_path << "\n\n";
        return false;
    }

    return true;
}

bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path);
    }

    return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "Cannot create directory:\n" << directory_path << "\n\n";
        return false;
    }

    std::cout << use_for << " output path:\n" << directory_path << "\n\n";
    return true;
}

bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "Cannot create directory:\n" << directory_path << "\n\n";
        return false;
    }

    return true;
}
}