FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/wsg_50_common/msg"
  "src/wsg_50_common/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/wsg_50_common/Conf.h"
  "srv_gen/cpp/include/wsg_50_common/Incr.h"
  "srv_gen/cpp/include/wsg_50_common/Move.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
