idf_component_register(
    SRCS
        "main.c"
        "ble_bp_client.c"
        "bp_parser.c"
        "zigbee_bp_device.c"
        "led_status.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        freertos
        driver
        espressif__esp-zigbee-lib
        espressif__led_strip
)
