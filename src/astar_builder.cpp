
#include "astar_builder.h"

#include "astar_param.h"
#include "yaml-cpp/yaml.h"

bool GetParam::getparam(const std::string& file)
{
    YAML::Node param                   = YAML::LoadFile(file);
    ExParam::Instance().succ_size_     = param["succ_size"].as<int>();
    ExParam::Instance().forward_size_  = param["forward_size"].as<int>();
    ExParam::Instance().backward_size_ = param["backward_size"].as<int>();

    // ExParam::Instance().delta_x_ = param["delta_x"].as<int>();
}
