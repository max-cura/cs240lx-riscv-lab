lab1.elf: force-cargo
	cargo build --profile release
	cp ../target/riscv32imac-unknown-none-elf/release/lab1 $@

lab1.uf2: lab1.elf
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
run: lab1.uf2
	cp lab1.uf2 $(MEDIA)/RP2350

clean:
	cargo clean -p lab1
	rm -f lab1.{elf,uf2}
