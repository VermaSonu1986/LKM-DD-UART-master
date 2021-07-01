#include <linux/build-salt.h>
#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x516e49f9, "module_layout" },
	{ 0x6c89514a, "no_llseek" },
	{ 0xf55bc851, "platform_driver_unregister" },
	{ 0xe89992a5, "__platform_driver_register" },
	{ 0x7c32d0f0, "printk" },
	{ 0xc4021e53, "_dev_err" },
	{ 0xbfd536c0, "misc_register" },
	{ 0xde8a7c17, "devm_kasprintf" },
	{ 0xccdf3fc8, "of_property_read_variable_u32_array" },
	{ 0xad31441d, "__pm_runtime_resume" },
	{ 0xab4f4b7b, "pm_runtime_enable" },
	{ 0x5bbe49f4, "__init_waitqueue_head" },
	{ 0x5d5ca740, "devm_request_threaded_irq" },
	{ 0xe9931696, "platform_get_irq" },
	{ 0x5bf8b57b, "devm_ioremap_resource" },
	{ 0x8b06ea8d, "devm_kmalloc" },
	{ 0x9f611f91, "platform_get_resource" },
	{ 0x5f754e5a, "memset" },
	{ 0x28cc25db, "arm_copy_from_user" },
	{ 0x822137e2, "arm_heavy_mb" },
	{ 0x3dcf1ffa, "__wake_up" },
	{ 0xdb7305a1, "__stack_chk_fail" },
	{ 0x49970de8, "finish_wait" },
	{ 0x647af474, "prepare_to_wait_event" },
	{ 0x1000e51, "schedule" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0xf4fa543b, "arm_copy_to_user" },
	{ 0x8f678b07, "__stack_chk_guard" },
	{ 0x11bbc3ee, "misc_deregister" },
	{ 0xeedde6ac, "__pm_runtime_disable" },
	{ 0x39a12ca7, "_raw_spin_unlock_irqrestore" },
	{ 0x5f849a69, "_raw_spin_lock_irqsave" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

