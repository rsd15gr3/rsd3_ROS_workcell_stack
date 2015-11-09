FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/wsg_50_common/msg"
  "src/wsg_50_common/srv"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/wsg_50_common/msg/__init__.py"
  "src/wsg_50_common/msg/_Status.py"
  "src/wsg_50_common/msg/_Cmd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
