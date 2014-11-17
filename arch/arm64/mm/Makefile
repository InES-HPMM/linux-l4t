GCOV_PROFILE := y

subdir-ccflags-y += -Werror

obj-y				:= dma-mapping.o extable.o fault.o init.o \
				   cache.o copypage.o flush.o \
				   ioremap.o mmap.o pgd.o mmu.o \
				   context.o tlb.o proc.o iomap.o

obj-$(CONFIG_ARM_PTDUMP)	+= dump.o
