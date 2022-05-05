cmake_minimum_required(VERSION 3.0.2)
project(cv_switch)
add_definitions(-std=c++14)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

# 如果你是不同显卡，请设置为显卡对应的号码参考这里：https://developer.nvidia.com/zh-cn/cuda-gpus#compute
#set(CUDA_GEN_CODE "-gencode=arch=compute_75,code=sm_75")
#set(OpenCV_DIR   "/data/datav/expstation/lean/opencv4.2.0/lib/cmake/opencv4/")
set(CUDA_DIR     "/usr/local/cuda-11.4")
set(CUDNN_DIR    "/usr/local/cuda-11.4/include")
set(TENSORRT_DIR "/usr/src/tensorrt")




#指定依赖包，生成对应的环境变量
find_package(catkin REQUIRED
        roscpp
        rospy
        geometry_msgs
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        message_generation
        dynamic_reconfigure
        xarm_api
        xarm_msgs
        )

find_package(OpenCV 3 REQUIRED)
find_package(CUDA REQUIRED)

find_package(PCL 1.12 REQUIRED)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )



#添加自定义消息
# add_message_files()
add_message_files(
    FILES
    object.msg
    multiobjects.msg
    BoundingBox.msg
    BoundingBoxes.msg
 )
#添加自定义服务
add_service_files(FILES serverSaveDetectionResult.srv)

#生成消息服务
generate_messages(
         DEPENDENCIES
         std_msgs
         geometry_msgs
         sensor_msgs
 )
#catkin宏命令，用于配置ROS的package配置文件和cmake文件，这个命令必须在add_library()和add_executable()之前调用，
# 有五个可选参数
# 1.INCLUDE_DIRS 导出包的includ路径
# 2.LIBRARIES-导出项目中的库
# 3.CATKIN_DEPENDS 该项目依赖的其他catkin项目
# 4.DEPENDS -该项目所依赖的非catkin cmake项目
# 5.CFG_EXTRAS  -其他配置选项
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES cv_switch
        CATKIN_DEPENDS roscpp rospy geometry_msgs sensor_msgs std_msgs dynamic_reconfigure message_runtime  xarm_msgs xarm_api
        #  DEPENDS system_lib
)


#指定c++的头文件路径
include_directories(
        include
        ${CUDA_DIR}/include
        ${TENSORRT_DIR}/include
        ${CUDNN_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tensorRT
        ${CMAKE_CURRENT_SOURCE_DIR}/tensorRT/common
        ${CMAKE_CURRENT_SOURCE_DIR}/tensorRT/infer
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
    
)

#指定链接库的路径
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${TENSORRT_DIR}/lib
    ${CUDA_DIR}/lib64
    ${CUDNN_DIR}/lib
)

# 指定SRC环境变量
set(SRC
        src/main.cpp
        src/cv_detection.cpp
	src/cv_segmentation.cpp
        src/deepsort.cpp
        src/simple_yolo.cu
        src/Algorithm_Objection_3D.cpp
        src/objection.cpp
        src/Position_Transform.cpp
        src/dataman.cpp
        tensorRT/common/cuda_tools.cpp
        tensorRT/common/ilogger.cpp
        tensorRT/common/json.cpp
        tensorRT/common/preprocess_kernel.cu
        tensorRT/common/trt_tensor.cpp
        tensorRT/infer/trt_infer.cpp
        )
# 指定可执行文件 有cuda文件和无cuda文件
# add_executable(${PROJECT_NAME} ${SRC})
cuda_add_executable(${PROJECT_NAME} ${SRC})
# 指定链接可执行文件
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})

target_include_directories(
    ${PROJECT_NAME}  PUBLIC ${CUDA_INCLUDE_DIRS} ${CUDNN_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(
    ${PROJECT_NAME}  -lnvinfer -lnvinfer_plugin -lnvparsers -lnvonnxparser
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
target_link_libraries(${PROJECT_NAME} "/usr/lib/x86_64-linux-gnu/libtiff.so")

target_link_libraries(${PROJECT_NAME} nvinfer nvinfer_plugin)
target_link_libraries(${PROJECT_NAME} cuda cublas cudart cudnn)
target_link_libraries(${PROJECT_NAME} protobuf pthread)
# 指定可执行依赖
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


# 安装目标文件到本地系统
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
