idf_component_register(SRCS "spl06.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus
                       PRIV_REQUIRES crazyflie platform)
