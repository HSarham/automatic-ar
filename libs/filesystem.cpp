#include "filesystem.h"

namespace filesystem {

std::vector<std::string> get_dirs_list(std::string path){
    std::vector<std::string> result;
    DIR *dir=opendir(path.c_str());

    if(dir!=NULL){
        for(dirent *entry=readdir(dir);entry!=NULL;entry=readdir(dir))
            if(entry->d_type == DT_DIR)
                result.push_back(entry->d_name);
        closedir(dir);
    }

    return result;
}

std::vector<std::string> get_files_list(std::string path){
    std::vector<std::string> result;
    DIR *dir=opendir(path.c_str());

    if(dir!=NULL){
        for(dirent *entry=readdir(dir);entry!=NULL;entry=readdir(dir))
            if(entry->d_type == DT_REG)
                result.push_back(entry->d_name);
        closedir(dir);
    }

    return result;
}

}


