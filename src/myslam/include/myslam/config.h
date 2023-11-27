#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"
#include <opencv2/core/core.hpp>

// 使用了类的单例模式，确保一个类只能有一个实例，并提供一个全局访问点来访问该实例
namespace myslam
{

    class Config
    {
    private:
        static std::shared_ptr<Config> config_; // 类的静态变量需要在实现中初始化，不能在声明处初始化
        cv::FileStorage file_;

        Config() {} // 将构造函数设为私有，以防止在外部创建实例

    public:
        ~Config();

        static bool setParameterFile(const std::string &filename);

        template<typename T>
        static T get(const std::string &key)
        {
            return T(Config::config_->file_[key]);
        }
    };

    std::shared_ptr<Config> Config::config_ = nullptr; // 初始化
}

#endif
