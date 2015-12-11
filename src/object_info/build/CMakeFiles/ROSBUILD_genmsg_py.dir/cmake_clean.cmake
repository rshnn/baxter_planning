FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/object_info/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/object_info/msg/__init__.py"
  "../src/object_info/msg/_ObjectInfo.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
