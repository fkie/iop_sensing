cmake_minimum_required(VERSION 3.0.2)
project(fkie_iop_measurement_sensor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
             fkie_iop_component
             fkie_iop_events
             fkie_iop_msgs
             roscpp
             )
iop_init(COMPONENT_ID 0)
iop_export_service(urn_jaus_jss_iop_MeasurementSensor)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        fkie_iop_component
        fkie_iop_events
        fkie_iop_msgs
        roscpp
)


###########
## Build ##
###########

iop_code_generator(
  IDLS
    urn.jaus.jss.core-v1.1/Events.xml
    urn.jaus.jss.core-v1.1/Transport.xml
    urn.jaus.jss.iop/MeasurementSensor.xml
  OWN_IDLS
  OVERRIDES
    include/urn_jaus_jss_iop_MeasurementSensor/MeasurementSensor_ReceiveFSM.h
    src/urn_jaus_jss_iop_MeasurementSensor/MeasurementSensor_ReceiveFSM.cpp
    src/main.cpp
  EXTERN_SERVICES
    urn_jaus_jss_core_Events
    urn_jaus_jss_core_Transport
  GENERATED_SOURCES cppfiles
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
            src/MeasurementSensorPlugin.cpp
            ${cppfiles}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${IOP_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
   FILES ./plugin_iop.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

