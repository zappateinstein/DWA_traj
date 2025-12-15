# Makefile for building flair demos using the CMake presets.
# Usage:
#   make          → build core2_64 (default)
#   make armv7a   → build ARMv7a NEON
#   make armv5    → build ARMv5te
#   make install  → install using default preset
#   make clean    → remove all build directories

.PHONY: all core2 armv7a armv5 install clean

all: core2

core2:
	cmake --preset flair-core2_64
	cmake --build --preset build-flair-core2_64

armv7a:
	cmake --preset flair-armv7a_neon
	cmake --build --preset build-flair-armv7a_neon

armv5:
	cmake --preset flair-armv5te
	cmake --build --preset build-flair-armv5te

install:
	cmake --build --preset build-flair-core2_64 --target install

clean:
	rm -rf build_core2_64 build_armv7a_neon build_armv5te
