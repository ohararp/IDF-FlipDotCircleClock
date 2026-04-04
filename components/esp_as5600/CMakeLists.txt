set(P_SOURCE
    "esp_as5600.c"
)

set(P_INCLUDE_DIR "include")

set(P_REQUIRES "")

set(P_PRIV_REQUIRES "esp_driver_i2c")

idf_component_register(
    SRCS ${P_SOURCE}
    INCLUDE_DIRS ${P_INCLUDE_DIR}
    REQUIRES ${P_REQUIRES}
    PRIV_REQUIRES ${P_PRIV_REQUIRES}
)