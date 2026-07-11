.PHONY: help format nix-format test build flash tui sim-images sim-test sim-format

SCENARIO ?=
VIEW     ?=

help:
	@echo "Firmware targets:"
	@echo "  format       clang-format all sources in-place + nixfmt + ruff format sim/"
	@echo "  nix-format   nixfmt all .nix files in-place"
	@echo "  test         run host Unity test suite (pio test -e native)"
	@echo "  build        compile firmware (pio run -e pico)"
	@echo "  flash        build and flash the RP2040 (pio run -e pico -t upload)"
	@echo ""
	@echo "Python sim targets (the native binding rebuilds on import as needed):"
	@echo "  tui          interactive picker; or SCENARIO=<name> [VIEW=timeseries|mounting|simulate]"
	@echo "  sim-images   regenerate docs/images screenshots (timeseries, mounting, simulate)"
	@echo "  sim-test     run sim pytest suite"
	@echo "  sim-format   ruff format + check in sim/"

format:
	find lib src test -type f \( -name "*.h" -o -name "*.cpp" \) -print0 \
		| xargs -0 clang-format -i
	$(MAKE) nix-format
	$(MAKE) sim-format

nix-format:
	find . -name "*.nix" -not -path "./.git/*" -print0 \
		| xargs -0 nixfmt

test:
	pio test -e native

build:
	pio run -e pico

flash:
	pio run -e pico -t upload

tui:
	cd sim && uv run --extra dev python -m plrs_sim $(if $(SCENARIO),sim $(SCENARIO) $(if $(VIEW),--view $(VIEW)))

sim-images:
	cd sim && MPLBACKEND=Agg uv run --extra dev python -m plrs_sim sim sinusoidal \
	    --view timeseries --no-show --save ../docs/images/sim-sinusoidal.png
	cd sim && MPLBACKEND=Agg uv run --extra dev python -m plrs_sim sim static \
	    --view mounting --no-show --save ../docs/images/sim-mounting.png
	cd sim && MPLBACKEND=Agg uv run --extra dev python -m plrs_sim sim wave_tack \
	    --view simulate --duration 50 --no-show --save ../docs/images/sim-simulate.png
	pngquant --force --skip-if-larger --ext .png --strip docs/images/sim-*.png

sim-test:
	cd sim && uv run --extra dev pytest

sim-format:
	cd sim && uv run ruff format . && uv run ruff check .
