okboot.elf: force-cargo
	cargo build --profile release
	cp ../target/riscv32imac-unknown-none-elf/release/okboot $@

okboot.uf2: okboot.elf
	engrave-uf2 $< -o $@

.DEFAULT_GOAL = run
.PHONY: force-cargo run clean

UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
	MEDIA := /media
endif
ifeq ($(UNAME),Darwin)
	MEDIA := /Volumes
endif
run: okboot.uf2
	cp okboot.uf2 $(MEDIA)/RP2350

clean:
	cargo clean -p okboot
	rm -f okboot.{elf,uf2}

test.o: test.S
	clang --target=riscv32 -march=rv32imac $^ -o $@ -c

test.elf: test.o | test.ld
	riscv64-elf-ld -melf32lriscv $^ -o $@ -Ttest.ld --gc-sections -z noexecstack -flto=thin -static

test.bin: test.elf
	riscv64-elf-objcopy $< -O binary $@
