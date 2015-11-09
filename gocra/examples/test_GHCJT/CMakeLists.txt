include_directories(
    ../../include
)

#========================================
add_library(
    Model3T
    SHARED
    Model3T.cpp
)

target_link_libraries(
    Model3T
    gocra
)

#========================================
add_executable(
    test_3T
    main.cpp
)

target_link_libraries(
    test_3T
    gocra
    Model3T
)
