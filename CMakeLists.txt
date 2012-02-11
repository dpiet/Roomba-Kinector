cmake_minimum_required(VERSION 2.6)

PROJECT(skeletor)

FIND_PACKAGE (Qt4 REQUIRED)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

QT4_WRAP_CPP(maoc_sources
  GuiController.h
  RawImagesWindow.h
)

QT4_WRAP_UI(uaic_sources
  RawImagesWindow.ui
)

# Default values
SET(NESTK_USE_FREENECT 1)
SET(NESTK_USE_OPENNI 1)

ADD_SUBDIRECTORY(nestk)
## nestk/UseNestk.cmake.in defines variable to use nestk.
## It itself includes nestk/deps/cmake/UseEmbeddedNestkDeps.cmake that
## contains most of the cmake find packages commands.
INCLUDE("${nestk_BINARY_DIR}/UseEmbeddedNestk.cmake")

SET(rgbd_skeletor_sources ${maoc_sources} ${uaic_sources}
               skeletor.cpp
               RawImagesWindow.h RawImagesWindow.cpp
               GuiController.cpp GuiController.h
               PoseRecognizer.cpp PoseRecognizer.h)

IF (BUILD_MACOSX_BUNDLE)
   ADD_EXECUTABLE(rgbd-skeletor MACOSX_BUNDLE ${rgbd_skeletor_sources}) 
   SET(DIRS ${QT_LIBRARY_DIRS} ${CMAKE_BINARY_DIR}/lib ${CMAKE_BINARY_DIR}/bin)
   INSTALL_STANDALONE_BUNDLE(rgbd-skeletor ${DIRS})
ELSE()
   ADD_EXECUTABLE(rgbd-skeletor ${rgbd_skeletor_sources})
ENDIF()

TARGET_LINK_LIBRARIES(rgbd-skeletor
                      nestk)
INSTALL(TARGETS rgbd-skeletor DESTINATION bin)

## ADD_EXECUTABLE(rgbd-skeletor ${rgbd_skeletor_sources})

## TARGET_LINK_LIBRARIES(rgbd-skeletor nestk)

