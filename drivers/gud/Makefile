#
# Makefile for the kernel mobicore drivers
#
GUD_ROOT_FOLDER := drivers/gud
# add our modules to kernel.
obj-$(CONFIG_MOBICORE_API) += mcKernelApi.o
obj-$(CONFIG_MOBICORE_SUPPORT) += mcDrvModule.o

<<<<<<< HEAD
mcDrvModule-objs := MobiCoreDriver/logging.o \
		MobiCoreDriver/ops.o \
		MobiCoreDriver/mem.o \
		MobiCoreDriver/api.o \
		MobiCoreDriver/pm.o \
		MobiCoreDriver/main.o

mcKernelApi-objs := MobiCoreKernelApi/main.o \
		MobiCoreKernelApi/clientlib.o \
		MobiCoreKernelApi/device.o \
		MobiCoreKernelApi/session.o \
		MobiCoreKernelApi/connection.o

# Release mode by default
ccflags-y := -DNDEBUG -I$(GUD_ROOT_FOLDER)
ccflags-y += -Wno-declaration-after-statement

#Netlink changed arguments number
ccflags-y += -DMC_NETLINK_COMPAT_V37

ccflags-$(CONFIG_MOBICORE_DEBUG) += -DDEBUG
ccflags-$(CONFIG_MOBICORE_VERBOSE) += -DDEBUG_VERBOSE

# Choose one platform from the folder
MOBICORE_PLATFORM := $(shell (ls -1 $(srctree)/$(GUD_ROOT_FOLDER)/MobiCoreDriver/platforms | tail -1) )
# Use the available platform folder
ccflags-y += -I$(GUD_ROOT_FOLDER)/MobiCoreDriver/platforms/$(MOBICORE_PLATFORM)
# MobiCore Driver includes
ccflags-y += -I$(GUD_ROOT_FOLDER)/MobiCoreDriver/public
# MobiCore KernelApi required incldes
ccflags-y += -I$(GUD_ROOT_FOLDER)/MobiCoreKernelApi/include
ccflags-y += -I$(GUD_ROOT_FOLDER)/MobiCoreKernelApi/public
=======
ifneq (,$(findstring N70,$(PRIVATE_RCMS_NAME)))
obj-$(CONFIG_TRUSTONIC_TEE) := 310d/MobiCoreDriver/
obj-$(CONFIG_TRUSTONIC_TRUSTED_UI) += 310d/TlcTui/
else
obj-$(CONFIG_TRUSTONIC_TEE) := 310a/MobiCoreDriver/
obj-$(CONFIG_TRUSTONIC_TRUSTED_UI) += 310a/TlcTui/
endif
>>>>>>> 0e91d2a... Nougat
