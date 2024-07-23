# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/uart.h)

zephyr_library()

if(CONFIG_ETC_UART_NATIVE_TTY)
  zephyr_library_compile_definitions(NO_POSIX_CHEATS)
  zephyr_library_sources(uart_native_tty.c)
  if (CONFIG_NATIVE_APPLICATION)
    zephyr_library_sources(uart_native_tty_bottom.c)
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
    target_sources(native_simulator INTERFACE uart_native_tty_bottom.c)
  endif()
endif()




