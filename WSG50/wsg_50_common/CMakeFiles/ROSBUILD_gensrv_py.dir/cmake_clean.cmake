FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/wsg_50_common/msg"
  "src/wsg_50_common/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/wsg_50_common/srv/__init__.py"
  "src/wsg_50_common/srv/_Conf.py"
  "src/wsg_50_common/srv/_Incr.py"
  "src/wsg_50_common/srv/_Move.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
