idf_component_register(
    SRCS "BoilerAutoFillMain.c" "BoilerAutoFillUI.c" "font_msyh_20.c" "font_sa_digital_number_64.c"
    REQUIRES esp_adc
    INCLUDE_DIRS "."
)
