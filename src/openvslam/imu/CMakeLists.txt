# Add sources
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bias.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/data.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/preintegrator.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/preintegrated.cc
)
