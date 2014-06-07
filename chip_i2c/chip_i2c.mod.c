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
	{ 0x5d8e4f32, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x81f463c, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0x467fa3e8, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xa3675d6f, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0x15fe8aa6, __VMLINUX_SYMBOL_STR(dev_set_drvdata) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x7713cbe4, __VMLINUX_SYMBOL_STR(devm_kzalloc) },
	{ 0x373db350, __VMLINUX_SYMBOL_STR(kstrtoint) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xca470ac7, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0xb1e8b154, __VMLINUX_SYMBOL_STR(dev_get_drvdata) },
	{ 0x2a5fbcaa, __VMLINUX_SYMBOL_STR(i2c_smbus_write_byte_data) },
	{ 0x91715312, __VMLINUX_SYMBOL_STR(sprintf) },
	{ 0x2abe1b70, __VMLINUX_SYMBOL_STR(dev_printk) },
	{ 0xc2de89b4, __VMLINUX_SYMBOL_STR(i2c_smbus_read_byte_data) },
	{ 0xefd6cf06, __VMLINUX_SYMBOL_STR(__aeabi_unwind_cpp_pr0) },
	{ 0x73e20c1c, __VMLINUX_SYMBOL_STR(strlcpy) },
	{ 0x676113f3, __VMLINUX_SYMBOL_STR(_dev_info) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:chip_i2c");

MODULE_INFO(srcversion, "A29EAD3E46B48959C7D3B6D");
