.PHONY: help format test build upload

help:
	@echo "Targets:"
	@echo "  format  clang-format all sources in-place"
	@echo "  test    run host Unity test suite (pio test -e native)"
	@echo "  build   compile firmware (pio run -e pico)"
	@echo "  upload  build and flash the RP2040 (pio run -e pico -t upload)"

format:
	find lib src test -type f \( -name "*.h" -o -name "*.cpp" \) -print0 \
		| xargs -0 clang-format -i

test:
	pio test -e native

build:
	pio run -e pico

upload:
	pio run -e pico -t upload
