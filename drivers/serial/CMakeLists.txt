# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/uart.h)

zephyr_library()

message("************************************************************************************************************")
message("CONFIG_ETC_UART_NAMED_PIPE_0=${CONFIG_ETC_UART_NAMED_PIPE_0}")
zephyr_library_sources_ifdef(CONFIG_ETC_UART_NAMED_PIPE_0	uart_named_pipe_0.c)


message("CONFIG_ETC_UART_NATIVE_TCP_0=${CONFIG_ETC_UART_NATIVE_TCP_0}")
zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TCP_0	uart_native_tcp_0.c)
message("CONFIG_ETC_UART_NATIVE_TCP_1=${CONFIG_ETC_UART_NATIVE_TCP_1}")
zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TCP_1	uart_native_tcp_1.c)
message("CONFIG_ETC_UART_NATIVE_TCP_2=${CONFIG_ETC_UART_NATIVE_TCP_2}")
zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TCP_2	uart_native_tcp_2.c)

if(CONFIG_ETC_UART_NATIVE_TTY OR CONFIG_ETC_UART_NATIVE_TCP )
message("CONFIG_ETC_UART_NATIVE_TTY=${CONFIG_ETC_UART_NATIVE_TTY}")
message("CONFIG_ETC_UART_NATIVE_TCP=${CONFIG_ETC_UART_NATIVE_TCP}")
message("CONFIG_NATIVE_APPLICATION=${CONFIG_NATIVE_APPLICATION}")
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)
  zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TTY		uart_native_tty.c)
  zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TCP		uart_native_tcp.c)

  
  if (CONFIG_NATIVE_APPLICATION)
  zephyr_library_sources_ifdef(CONFIG_ETC_UART_NATIVE_TTY		uart_native_tty.c)
    if(CONFIG_ETC_UART_NATIVE_TTY)
        zephyr_library_sources(uart_native_tty_bottom.c)
    endif()
    if(CONFIG_ETC_UART_NATIVE_TCP OR CONFIG_ETC_UART_NATIVE_TCP_1)
        zephyr_library_sources(uart_native_tcp_bottom.c)
    endif()
    zephyr_include_directories(nsi_compat/)
    zephyr_include_directories(${ZEPHYR_BASE}/scripts/native_simulator/common/src/include/)
    zephyr_library_sources(
      posix_core.c
      nsi_compat/nsi_compat.c
      ${ZEPHYR_BASE}/scripts/native_simulator/common/src/nce.c
      ${ZEPHYR_BASE}/scripts/native_simulator/common/src/nsi_host_trampolines.c
    )

	zephyr_library_compile_definitions(_POSIX_C_SOURCE=200809L _XOPEN_SOURCE=600 _XOPEN_SOURCE_EXTENDED)
  else()
     if(CONFIG_ETC_UART_NATIVE_TTY)
         target_sources(native_simulator INTERFACE uart_native_tty_bottom.c)
     endif()
     if(CONFIG_ETC_UART_NATIVE_TCP OR CONFIG_ETC_UART_NATIVE_TCP_1)
        target_sources(native_simulator INTERFACE uart_native_tcp_bottom.c)
     endif()
  endif()
endif()





