set(EXEC hello_carousel_cube.bin)
set(SRCS hello_carousel_cube.c)

add_executable(${EXEC} ${SRCS})
target_link_libraries(${EXEC} ${HELLO_PI_LIBS})

install(TARGETS ${EXEC}
        RUNTIME DESTINATION bin)
