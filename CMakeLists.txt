project(pvr.demo)

cmake_minimum_required(VERSION 2.6)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

enable_language(CXX)

find_package(kodi REQUIRED)
find_package(kodiplatform REQUIRED)
find_package(p8-platform REQUIRED)

add_definitions(-DUSE_DEMUX)

include_directories(${kodiplatform_INCLUDE_DIRS}
                    ${p8-platform_INCLUDE_DIRS}
                    ${KODI_INCLUDE_DIR}
                    ${PROJECT_SOURCE_DIR}/src
                    ${PROJECT_SOURCE_DIR}/src/teststream)

set(DEPLIBS ${kodiplatform_LIBRARIES}
            ${p8-platform_LIBRARIES})

set(PVRDEMO_SOURCES src/client.cpp
                    src/PVRDemoData.cpp
                    src/teststream/bitstream.cpp
                    src/teststream/demuxer.cpp
                    src/teststream/parser.cpp
                    src/teststream/parser_AAC.cpp
                    src/teststream/parser_AC3.cpp
                    src/teststream/parser_DTS.cpp
                    src/teststream/parser_h264.cpp
                    src/teststream/parser_MPEGAudio.cpp
                    src/teststream/parser_MPEGVideo.cpp
                    src/teststream/parser_Subtitle.cpp
                    src/teststream/parser_Teletext.cpp
                    src/teststream/streamer.cpp
                    src/teststream/videobuffer.cpp
                    src/teststream/libsi/descriptor.cpp
                    src/teststream/libsi/section.cpp
                    src/teststream/libsi/si.cpp
                    src/teststream/libsi/util.cpp)

build_addon(pvr.demo PVRDEMO DEPLIBS)

include(CPack)
