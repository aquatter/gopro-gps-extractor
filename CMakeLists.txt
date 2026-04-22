cmake_minimum_required(VERSION 3.15.0)
project(gopro_gps_extractor LANGUAGES CXX)

find_package(fmt CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(range-v3 CONFIG REQUIRED)
find_package(ng-log CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(tinyxml2 CONFIG REQUIRED)

add_executable(gopro_gps_extractor 
    main.cpp
    src/progress_bar.cpp
    src/gpmf_parser.cpp
    src/gpmf_frame.cpp
    gpmf-parser/demo/GPMF_mp4reader.c
    gpmf-parser/demo/GPMF_print.c
    gpmf-parser/GPMF_parser.c
    gpmf-parser/GPMF_utils.c
    )

set_target_properties(gopro_gps_extractor PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)

target_include_directories(gopro_gps_extractor PRIVATE
    include
    gpmf-parser
    gpmf-parser/demo
    portable-file-dialogs
)

target_link_libraries(gopro_gps_extractor PRIVATE 
    fmt::fmt
    imgui::imgui
    Eigen3::Eigen
    glfw
    range-v3::range-v3
    ng-log::ng-log    
    tinyxml2::tinyxml2
    d3d11.lib
    nlohmann_json::nlohmann_json
)