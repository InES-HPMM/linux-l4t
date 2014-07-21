GCOV_PROFILE := y

ccflags-y := -Werror
obj-y := nvadsp.o
nvadsp-objs += dev.o os.o app.o app_loader_linker.o\
	 aram.o nvadsp_dram.o \
	 nvadsp_shared_sema.o nvadsp_arb_sema.o \
	 hwmailbox.o mailbox.o ape_actmon.o msgq.o \
	 mem_manager.o aram_manager.o dram_app_mem_manager.o

ifeq ($(CONFIG_TEGRA_ADSP_DFS),y)
nvadsp-objs += adsp_dfs.o
endif
