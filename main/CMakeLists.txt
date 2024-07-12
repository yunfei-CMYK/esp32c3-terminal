file(GLOB_RECURSE SRC_SOURCES ../ui_smart_gadget/components/*.c ../ui_smart_gadget/fonts/*.c ../ui_smart_gadget/images/*.c ../ui_smart_gadget/screens/*.c)

idf_component_register(SRCS "myi2c.c" "spi_lcd_touch_example_main.c" "lvgl_demo_ui.c"
                       SRCS "../ui_smart_gadget/ui.c" "../ui_smart_gadget/ui_helpers.c" ${SRC_SOURCES}
                       INCLUDE_DIRS "." "../ui_smart_gadget" )
