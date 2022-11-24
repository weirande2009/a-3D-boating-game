#ifndef FILESYSTEM_H
#define FILESYSTEM_H
#define _CRT_SECURE_NO_WARNINGS
#include <cstdlib>
#include <string>

// TODO 改成项目的绝对路径（原来是用CMAKE生成的）
const char* logl_root = "";

class FileSystem {
private:
    typedef std::string (*Builder)(const std::string& path);

public:
    static std::string getPath(const std::string& path)
    {
        static std::string (*pathBuilder)(std::string const&) = getPathBuilder();
        return (*pathBuilder)(path);
    }

private:
    static std::string const& getRoot()
    {
        //static char const* envRoot = getenv("LOGL_ROOT_PATH");
        static char* envRoot;
        static size_t len;
        errno_t err = _dupenv_s(&envRoot, &len, "LOGL_ROOT_PATH");
        if (err)envRoot = nullptr;
        static char const* givenRoot = (envRoot != nullptr ? envRoot : logl_root);
        static std::string root = (givenRoot != nullptr ? givenRoot : "");
        return root;
    }

    //static std::string(*foo (std::string const &)) getPathBuilder()
    static Builder getPathBuilder()
    {
        if (getRoot() != "")
            return &FileSystem::getPathRelativeRoot;
        else
            return &FileSystem::getPathRelativeBinary;
    }

    static std::string getPathRelativeRoot(const std::string& path)
    {
        return getRoot() + std::string("/") + path;
    }

    static std::string getPathRelativeBinary(const std::string& path)
    {
        return path;
    }
};

// FILESYSTEM_H
#endif
