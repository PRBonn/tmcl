
add_executable(TextRecoFromFolder TextRecoFromFolder.cpp TextSpotting.cpp)
target_link_libraries(TextRecoFromFolder ${OpenCV_LIBS} ${Boost_LIBRARIES} )

add_executable(TextRecoFromFolder2 TextRecoFromFolder2.cpp TextSpotting.cpp)
target_link_libraries(TextRecoFromFolder2 ${OpenCV_LIBS} ${Boost_LIBRARIES} )


add_library(NDL TextSpotting.cpp ImageUtils.cpp)





