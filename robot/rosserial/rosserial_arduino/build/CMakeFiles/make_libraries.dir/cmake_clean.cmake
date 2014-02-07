FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/rosserial_arduino/msg"
  "../msg_gen"
  "CMakeFiles/make_libraries"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/make_libraries.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
