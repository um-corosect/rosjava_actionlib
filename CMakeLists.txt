##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rosjava_actionlib)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
#catkin_rosjava_setup(installApp publishMavenJavaPublicationToMavenRepository)
catkin_rosjava_setup(install publishMavenJavaPublicationToMavenRepository)

catkin_package()

##############################################################################
# Installation
##############################################################################

# Install the launchfiles
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
