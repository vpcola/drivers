#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x146fd334, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x3ce4ca6f, __VMLINUX_SYMBOL_STR(disable_irq) },
	{ 0xca470ac7, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0xefea843e, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xc2de89b4, __VMLINUX_SYMBOL_STR(i2c_smbus_read_byte_data) },
	{ 0x2e5810c6, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr1) },
	{ 0xc068440e, __VMLINUX_SYMBOL_STR(__kfifo_alloc) },
	{ 0x15fe8aa6, __VMLINUX_SYMBOL_STR(dev_set_drvdata) },
	{ 0x5d8e4f32, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x92a9c60c, __VMLINUX_SYMBOL_STR(time_to_tm) },
	{ 0x2abe1b70, __VMLINUX_SYMBOL_STR(dev_printk) },
	{ 0x2a5fbcaa, __VMLINUX_SYMBOL_STR(i2c_smbus_write_byte_data) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x173c7e2c, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0x62b72b0d, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x467fa3e8, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x4578f528, __VMLINUX_SYMBOL_STR(__kfifo_to_user) },
	{ 0x48a0f939, __VMLINUX_SYMBOL_STR(mutex_lock_interruptible) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x73e20c1c, __VMLINUX_SYMBOL_STR(strlcpy) },
	{ 0xd7b094de, __VMLINUX_SYMBOL_STR(noop_llseek) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0x81f463c, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xa3675d6f, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0x676113f3, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xde6b4721, __VMLINUX_SYMBOL_STR(kmem_cache_alloc) },
	{ 0xf0fdf6cb, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xdb760f52, __VMLINUX_SYMBOL_STR(__kfifo_free) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0x4f68e5c9, __VMLINUX_SYMBOL_STR(do_gettimeofday) },
	{ 0xc66123b2, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0xfcec0987, __VMLINUX_SYMBOL_STR(enable_irq) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0xf23fcb99, __VMLINUX_SYMBOL_STR(__kfifo_in) },
	{ 0x8f678b07, __VMLINUX_SYMBOL_STR(__stack_chk_guard) },
	{ 0xb1e8b154, __VMLINUX_SYMBOL_STR(dev_get_drvdata) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:mpl3115");

MODULE_INFO(srcversion, "D3C862E1A9230845F1E2C6B");
