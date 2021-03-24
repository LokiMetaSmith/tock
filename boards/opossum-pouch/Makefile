# Makefile for building the tock kernel for the Opossum Pouch

TOCK_ARCH=cortex-m4
TARGET=thumbv7em-none-eabi
PLATFORM=opossum-pouch

include ../Makefile.common
TOCKLOADER=tockloader
OPENOCD=openocd
OPENOCD_OPTIONS=-f openocd_nrf52.cfg
# Where in the nrf52 flash to load the kernel with `tockloader`
KERNEL_ADDRESS=0x00000


APP=../../../libtock-c/examples/sensors/build/cortex-m4/cortex-m4.tbf
KERNEL=$(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
KERNEL_WITH_APP=$(TOCK_ROOT_DIRECTORY)/target/$(TARGET)/release/$(PLATFORM)-app.elf

# Upload programs over uart with tockloader
ifdef PORT
  TOCKLOADER_GENERAL_FLAGS += --port $(PORT)
endif

TOCKLOADER_JTAG_FLAGS = --jlink --board nrf52dk

.PHONY: flash-debug
flash-debug: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/debug/$(PLATFORM).elf
	$(OPENOCD) $(OPENOCD_OPTIONS) -c "program $<; verify_image $<;  reset; shutdown;"


# Upload the kernel over JTAG

.PHONY: flash-segger
flash-segger: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(TOCKLOADER) $(TOCKLOADER_GENERAL_FLAGS) flash --address $(KERNEL_ADDRESS) $(TOCKLOADER_JTAG_FLAGS) $<


.PHONY: flash-ocd
flash-ocd: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).elf
	$(OPENOCD) $(OPENOCD_OPTIONS) -c "program $<; verify_image $<; reset; shutdown;"

# Upload the kernel over JTAG using OpenOCD
.PHONY: flash-openocd
flash-openocd: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	$(TOCKLOADER) $(TOCKLOADER_GENERAL_FLAGS) flash --address $(KERNEL_ADDRESS) --board nrf52dk  --openocd-commands 'adapter speed 1000' --openocd $<

.PHONY: flash-openocd-debug
flash-openocd-debug:
# Upload the kernel over serial/bootloader
.PHONY: program-usb
program-usb: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).hex
	$(error Cannot program nRF52840-Dongle over USB. Use \`make flash\` and JTAG)

.PHONY: program
program: $(TOCK_ROOT_DIRECTORY)target/$(TARGET)/release/$(PLATFORM).bin
	arm-none-eabi-objcopy --update-section .apps=$(APP) $(KERNEL) $(KERNEL_WITH_APP)
	$(OPENOCD) $(OPENOCD_OPTIONS) -c "program $(KERNEL_WITH_APP); verify_image $(KERNEL_WITH_APP); reset; shutdown;"

.PHONY: erase
#	tockloader flash --board nrf52dk --openocd ~/Projects/tock/target/thumbv7em-none-eabi/release/nrf52840_dongle.bin
erase:	$(OPENOCD) $(OPENOCD_OPTIONS) -c "adapter speed 1000; flash fillb 0x30000 0xff 512; reset; shutdown;"
