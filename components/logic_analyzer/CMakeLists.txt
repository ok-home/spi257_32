set( srcs 
    logic_analyzer_hal/logic_analyzer_triggered_isr_start.c
    logic_analyzer_hal/logic_analyzer_hal.c
    logic_analyzer_hal/logic_analyzer_ll.c
    logic_analyzer_hal/logic_analyzer_hi_interrupt_handler.s
)
set( includedir logic_analyzer_hal/include )
set( require 
    driver
    protocol_examples_common
    esp_http_server 
    esp_wifi 
    nvs_flash 
    esp_eth    
    )

if(CONFIG_ANALYZER_USE_WS)
    list( APPEND srcs
        logic_analyzer_ws/logic_analyzer_ws.c
        logic_analyzer_ws/logic_analyzer_ws_server.c
    )
    list( APPEND includedir 
        logic_analyzer_ws/include 
    )
    set(embed  
        logic_analyzer_ws/logic_analyzer_ws.html
    )
endif()

if(CONFIG_ANALYZER_USE_SUMP)
    list( APPEND srcs
        logic_analyzer_sump/logic_analyzer_sump.c
    )
    list( APPEND includedir 
        logic_analyzer_sump/include 
    )
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${includedir}
    REQUIRES ${require}
    EMBED_FILES ${embed}
)

if(CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_4)
        target_link_libraries(${COMPONENT_LIB} INTERFACE "-u la_include_hi_interrupt")
endif()