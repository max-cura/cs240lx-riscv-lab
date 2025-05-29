.SUFFIXES:

simple.o: simple.S
	clang --target=riscv32 -march=rv32imac $< -o $@ -c

simple.elf: simple.o | simple.ld
	riscv64-unknown-elf-ld -melf32lriscv $^ -o $@ -Tsimple.ld --gc-sections -z noexecstack -flto=thin -static

simple.uf2: simple.elf
	cargo run --bin engrave-uf2 simple.elf -o simple.uf2

clean:
	rm -f simple.{o,elf,uf2}

.PHONY: clean

UNAME := $(shell uname -s)
ifeq ($(UNAME),Linux)
	MEDIA := /media
endif
ifeq ($(UNAME),Darwin)
	MEDIA := /Volumes
endif
run: simple.uf2
	cp simple.uf2 $(MEDIA)/RP2350

.DEFAULT_GOAL := run
.PHONY: run