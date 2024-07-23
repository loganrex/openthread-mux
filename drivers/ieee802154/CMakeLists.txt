# SPDX-License-Identifier: Apache-2.0

zephyr_library()

message( "CONFIG_IEEE802154_ETC_UPIPE ${CONFIG_IEEE802154_ETC_UPIPE} ************************************************" )

zephyr_library_sources_ifdef(CONFIG_IEEE802154_ETC_UPIPE		ieee802154_etc_uart_pipe.c)


