extern struct smp_operations tegra_smp_ops;

extern phys_addr_t tegra_tsec_start;
extern phys_addr_t tegra_tsec_size;

#ifdef CONFIG_TEGRA_USE_SECURE_KERNEL
extern unsigned long tegra_tzram_start;
extern unsigned long tegra_tzram_size;
#endif

#ifdef CONFIG_CACHE_L2X0
void tegra_init_cache(bool init);
#else
static inline void tegra_init_cache(bool init) {}
#endif

extern void tegra_cpu_die(unsigned int cpu);
extern int tegra_cpu_kill(unsigned int cpu);
extern phys_addr_t tegra_avp_kernel_start;
extern phys_addr_t tegra_avp_kernel_size;
void ahb_gizmo_writel(unsigned long val, void __iomem *reg);

extern struct device tegra_generic_cma_dev;
extern struct device tegra_vpr_cma_dev;
