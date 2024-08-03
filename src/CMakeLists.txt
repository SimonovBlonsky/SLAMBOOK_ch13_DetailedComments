add_library(myslam SHARED
        Frame.cpp
        MapPoint.cpp
        Map.cpp
        Camera.cpp
        Config.cpp
        Feature.cpp
        Frontend.cpp
        Backend.cpp
        Viewer.cpp
        VisualOdometry.cpp
        Dataset.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})