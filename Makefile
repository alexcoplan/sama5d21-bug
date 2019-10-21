# To compile for sama5d21, set CC to point to arm/atmel gcc.
#
# To compile for both arches, run:
# make
# make CC=/path/to/atmel/gcc

BUILD_DIR := build/$(shell $(CC) -dumpmachine)

.PHONY: clean

$(BUILD_DIR)/serial_stress: serial_stress.c $(BUILD_DIR)
	$(CC) -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf build
