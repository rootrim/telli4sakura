idf_component_register(
    SRCS
        "ms5611_drv.c"
        "bmp390_drv.c"
        "mpu6050_drv.c"
        "gps_drv.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp-idf-lib__ms5611
        k0i05__esp_bmp390
        esp-idf-lib__mpu6050
        minmea
        esp_driver_uart
)
 
