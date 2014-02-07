FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rosserial_arduino/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rosserial_arduino/msg/__init__.py"
  "../src/rosserial_arduino/msg/_Adc.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
