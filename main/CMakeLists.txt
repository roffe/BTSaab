idf_component_register(SRCS "btsaab_main.cpp"
    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
