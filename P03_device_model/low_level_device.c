/* Assignment for device model */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "SPI_PLDRV"

//TODO 3.9: Intialize the start address and end address for SPI 0
#define RESOURCE1_START_ADDRESS 0x00000000
#define RESOURCE1_END_ADDRESS	0x00000000
//TODO 3.10: Intialize the base address for the SPI 0 Pad register
#define RESOURCE2_START_ADDRESS 0x00000000
#define RESOURCE2_END_ADDRESS	0x00000000

/* Specifying my resources information */
//TODO 3.11: Populate the memory resource
static struct resource sample_resources[] = {
	{
	},
};


//TODO 3.12: Populate the platform device structure
static struct platform_device sample_device = 
{
};

static __init int init_platform_dev(void)
{
	printk("sample Platform driver(device).... \n");
	//TODO 3.13: Register the platform device
	return 0;
}

static void __exit exit_platform_dev(void)
{
	//TODO 3.15: Un-register the platform device
	printk("Exiting sample Platform(device) driver... \n");
}

module_init(init_platform_dev);
module_exit(exit_platform_dev);

MODULE_AUTHOR("Embitude Trainings <info@embitude.in>");
MODULE_DESCRIPTION("Sample Platform Driver");
MODULE_LICENSE("GPL");
