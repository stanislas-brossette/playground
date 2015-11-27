set(SOURCES
  main.cpp
  )
set(HEADERS
)

set(LIBRARIES
  YamlTest
)
foreach(lib ${LIBRARIES})
  add_library(${lib} SHARED ${SOURCES} ${HEADERS})
  set_target_properties(${lib} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0.1
  )
  target_compile_definitions(${lib} PUBLIC "-DPGS_EXPORT")
  PKG_CONFIG_USE_DEPENDENCY(${lib} "yaml-cpp")
endforeach()


install(TARGETS YamlTest
  RUNTIME DESTINATION ${CMAKE_INSTALL_FULL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR}
)

add_executable(YamlTestExe main.cpp)
target_link_libraries(YamlTestExe YamlTest)
