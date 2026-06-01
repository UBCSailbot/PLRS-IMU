.PHONY: help format test build upload sim sim-test sim-format sim-example

SCENARIO ?= constant_turn
EXAMPLE  ?= constant_turn

help:
	@echo "Firmware targets:"
	@echo "  format       clang-format all sources in-place"
	@echo "  test         run host Unity test suite (pio test -e native)"
	@echo "  build        compile firmware (pio run -e pico)"
	@echo "  upload       build and flash the RP2040 (pio run -e pico -t upload)"
	@echo ""
	@echo "Python sim targets:"
	@echo "  sim          run CLI scenario (SCENARIO=constant_turn|sinusoidal|step_turns|static)"
	@echo "  sim-example  run example script (EXAMPLE=constant_turn|sinusoidal|step_turns|static)"
	@echo "  sim-test     run sim pytest suite"
	@echo "  sim-format   ruff format + check in sim/"

format:
	find lib src test -type f \( -name "*.h" -o -name "*.cpp" \) -print0 \
		| xargs -0 clang-format -i

test:
	pio test -e native

build:
	pio run -e pico

upload:
	pio run -e pico -t upload

sim:
	cd sim && uv run python -m plrs_sim sim $(SCENARIO)

sim-example:
	cd sim && uv run python examples/$(EXAMPLE).py

sim-test:
	cd sim && uv run pytest

sim-format:
	cd sim && uv run ruff format . && uv run ruff check .
