FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/rosserial_msgs/msg"
  "../src/rosserial_msgs/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rosserial_msgs/TopicInfo.h"
  "../msg_gen/cpp/include/rosserial_msgs/Log.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
