#include <gflags/gflags.h>
#include "myslam/VisualOdometry.h"

// DEFINE_STRING(name, default value, “explaination”)
DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}