// KGSL FOR ZTE ARTHUR
// DM47021
// UPDATE: ICS CHOCOLATE FOR JELLYBEAN 
// KERNEL 2.6.35.7-perf+

#include <linux/msm_kgsl.h>

struct resource kgsl_3d0_resources[] = {
	{
		.name  = KGSL_3D0_REG_MEMORY,
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_3D0_IRQ,
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_3d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 280000000,
				.bus_freq = 224000000,
			},
			{
				.gpu_freq = 192000000,
				.bus_freq = 152000000,
			},
			{
				.gpu_freq = 192000000,
				.bus_freq = 0,
			},
		},
		.init_level = 0,
		.num_levels = 3,
		.set_grp_async = set_grp3d_async,
		.idle_timeout = HZ/20,
		.nap_allowed = true,
		.idle_needed = true,
	},
	.clk = {
		.name = {
			.clk = "grp_clk",
			.pclk = "grp_pclk",
		},
	},
	.imem_clk_name = {
		.clk = "imem_clk",
		.pclk = NULL,
	},
};

struct platform_device msm_kgsl_3d0 = {
	.name = "kgsl-3d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_3d0_resources),
	.resource = kgsl_3d0_resources,
	.dev = {
		.platform_data = &kgsl_3d0_pdata,
	},
};

static struct resource kgsl_2d0_resources[] = {
	{
		.name = KGSL_2D0_REG_MEMORY,
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = KGSL_2D0_IRQ,
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_device_platform_data kgsl_2d0_pdata = {
	.pwr_data = {
		.pwrlevel = {
			{
				.gpu_freq = 0,
				.bus_freq = 192000000,
			},
		},
		.init_level = 0,
		.num_levels = 1,
		/* HW workaround, run Z180 SYNC @ 192 MHZ */
		.set_grp_async = NULL,
		.idle_timeout = HZ/10,
		.nap_allowed = true,
		.idle_needed = true,
	},
	.clk = {
		.name = {
			.clk = "grp_2d_clk",
			.pclk = "grp_2d_pclk",
		},
	},
};

struct platform_device msm_kgsl_2d0 = {
	.name = "kgsl-2d0",
	.id = 0,
	.num_resources = ARRAY_SIZE(kgsl_2d0_resources),
	.resource = kgsl_2d0_resources,
	.dev = {
		.platform_data = &kgsl_2d0_pdata,
	},
};

