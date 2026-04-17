idf_component_register(SRCS ./app_driver.c 
                            ./app_main.c
                            ./MFRC522.c
                            ./keypad_i2c.c
                            ./card_storage.c
                            ./oled_i2c.c
                       INCLUDE_DIRS ".")
