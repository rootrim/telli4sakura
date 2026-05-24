idf_component_register(
    SRCS "fcc.c"
    INCLUDE_DIRS "."
    REQUIRES
        sensors
        filters
        flight_state
        lora
)
