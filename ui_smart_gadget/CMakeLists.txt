SET(SOURCES screens/ui_Startpage.c
    screens/ui_Mainpage.c
    screens/ui_weatherpage.c
    screens/ui_gamepage.c
    screens/ui_portpage.c
    screens/ui_mpupage.c
    screens/ui_campasspage.c
    screens/ui_settingpage.c
    screens/ui_wifipage.c
    screens/ui_netapp.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_wifi_png.c
    images/ui_img_944254084.c
    images/ui_img_227306801.c
    images/ui_img_2114266571.c
    images/ui_img_613467726.c
    images/ui_img_499049799.c
    images/ui_img_1887403499.c
#   images/image_taikong.c
    fonts/font_alipuhui.c
    fonts/desktopweather.c
    fonts/font_led.c
    fonts/myawesome.c
    fonts/qweather.c
    fonts/ui_font_Terminal.c
    fonts/ui_font_Terminal22.c)

add_library(ui ${SOURCES})
