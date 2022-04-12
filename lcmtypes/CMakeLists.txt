
find_package(lcm REQUIRED)
include(${LCM_USE_FILE})

lcm_wrap_types(
  CPP_HEADERS cpp_headers
  lcm_std_msgs.lcm
)

lcm_add_library(lcm-types-cpp CPP ${cpp_headers})
target_include_directories(lcm-types-cpp INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
