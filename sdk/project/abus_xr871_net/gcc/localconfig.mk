__PRJ_CONFIG_BOARD := abus_ppic90520
__PRJ_CONFIG_XIP := n
ifeq ($(__PRJ_CONFIG_XIP), y)
export __CONFIG_XIP_SECTION_FUNC_LEVEL := y
endif
# no float printf/scanf: none of our code formats floats, saves ~10 KB
__CONFIG_LIBC_PRINTF_FLOAT := n
__CONFIG_LIBC_SCANF_FLOAT := n
