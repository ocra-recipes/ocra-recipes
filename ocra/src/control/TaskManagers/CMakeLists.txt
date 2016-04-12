file(GLOB ProtoFiles ${CMAKE_CURRENT_SOURCE_DIR}/TaskManagerOptions.proto )

set(ProtoSources "" PARENT_SCOPE)
set(Protoheaders "" PARENT_SCOPE)
foreach(ProtoFile ${ProtoFiles})
    get_filename_component(proto ${ProtoFile} NAME)
    
    CONFIGURE_FILE(${ProtoFile}
                    ${CMAKE_CURRENT_BINARY_DIR}/${proto}
                    COPYONLY)
                    
    message("ProtoFile :  ${ProtoFile}")

    list(APPEND ProtoSources ${ProtoSource})
    list(APPEND ProtoHeaders ${Protoheader})
endforeach()


PROTOBUF_GENERATE_CPP(ProtoSources Protoheaders ${CMAKE_CURRENT_BINARY_DIR} ${ProtoFiles})
add_custom_target(${proto}_protobuf_output ALL DEPENDS ${ProtoSources} ${ProtoHeaders})
    
message("
${CMAKE_CURRENT_BINARY_DIR}/control/TaskManagers
Protobufs : 

${ProtoSources}

${ProtoHeaders}

")