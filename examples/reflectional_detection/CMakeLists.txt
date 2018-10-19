add_executable (reflectional_detection main.cpp )
target_link_libraries (reflectional_detection ${PCL_LIBRARIES} ${Boost_LIBRARIES} reflectional_symmetry_detection_example_options ${PROTOBUF_LIBRARY})
