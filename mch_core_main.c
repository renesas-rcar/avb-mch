#include <linux/module.h>
#include <linux/init.h>

static int __init mch_module_init(void)
{
	/* stub */
	return 0;
}

static void __exit mch_module_exit(void)
{
	/* stub */
}

module_init(mch_module_init);
module_exit(mch_module_exit);

MODULE_AUTHOR("Renesas Electronics Corporation");
MODULE_DESCRIPTION("Renesas Media Clock recovery Handler");
MODULE_LICENSE("Dual MIT/GPL");
