#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/compile.h>

static int buildinfo_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", BLDINFO);
	return 0;
}

static int buildinfo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, buildinfo_proc_show, NULL);
}

static const struct file_operations buildinfo_proc_fops = {
	.open		= buildinfo_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_buildinfo_init(void)
{
	proc_create("buildinfo", 0, NULL, &buildinfo_proc_fops);
	return 0;
}
module_init(proc_buildinfo_init);
