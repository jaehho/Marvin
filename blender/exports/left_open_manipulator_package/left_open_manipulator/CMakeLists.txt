cmake_minimum_required(VERSION 2.8.3)

project(marvin)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

foreach(dir . meshes meshes/stl urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
