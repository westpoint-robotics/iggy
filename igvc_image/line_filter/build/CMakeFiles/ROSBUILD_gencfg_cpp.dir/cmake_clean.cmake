FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/line_filter/LineFilterConfig.h"
  "../docs/LineFilterConfig.dox"
  "../docs/LineFilterConfig-usage.dox"
  "../src/line_filter/cfg/LineFilterConfig.py"
  "../docs/LineFilterConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
