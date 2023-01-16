idf_component_register(SRCS "btsaab_comp.cpp" "btsaab_can.cpp" "btsaab_sid.cpp" "utf_convert.cpp" "btsaab_helpers.cpp" "btsaab_main.cpp"
    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
