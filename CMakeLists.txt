cmake_minimum_required(VERSION 2.8)
project( test )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( optical_2 optical_flow.cpp  )
target_link_libraries( optical_2 ${OpenCV_LIBS} )


add_executable( aruco aruco.cpp  )
target_link_libraries( aruco ${OpenCV_LIBS} )

add_executable( aruco_board aruco_board.cpp  )
target_link_libraries( aruco_board ${OpenCV_LIBS} )

#add_executable( pose_est pose_est.cpp  )
#target_link_libraries( pose_est ${OpenCV_LIBS} )