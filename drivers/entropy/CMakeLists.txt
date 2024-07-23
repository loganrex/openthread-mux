zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/entropy.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_FAKE_ENTROPY_QEMU         fake_entropy_qemu.c)

