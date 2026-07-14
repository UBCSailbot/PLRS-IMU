{
  description = "POLARIS IMU Development Shell";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
  };

  outputs =
    { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          clang-tools
          cmake
          jq
          ninja
          nixfmt
          platformio
          pngquant
          python3
          ruff
          uv
        ];
        # PyPI wheels link against system libs that on Nix only live in the
        # store. Expose them via LD_LIBRARY_PATH. (FHS is the alternative but
        # `nix develop -c` does not enter its namespace.)
        #
        # stdenv.cc.cc.lib + zlib: libstdc++/libgcc for numpy and the SciPy
        # stack.
        # zstd, glib, fontconfig, freetype, dbus, libxkbcommon, xorg.libX11,
        # libGL: system libs that the PyPI PyQt6 wheel's bundled Qt6 links
        # against transitively (Qt6Core, Qt6Gui, Qt6Widgets).
        # wayland, xcb-util-cursor: needed by the Qt platform plugins
        # (libqwayland.so wants libwayland-client.so.0; the xcb fallback wants
        # libxcb-cursor). These only load when an actual window opens, so the
        # offscreen GIF path (plot_animate) never needed them -- the live
        # monitor's interactive QtAgg window is the first thing that does.
        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
          pkgs.stdenv.cc.cc.lib
          pkgs.zlib
          pkgs.zstd
          pkgs.glib
          pkgs.fontconfig
          pkgs.freetype
          pkgs.dbus
          pkgs.libxkbcommon
          pkgs.libx11
          pkgs.libGL
          pkgs.wayland
          pkgs.xcb-util-cursor
        ];
        # compiledb generation and .clangd setup live in
        # scripts/setup-compile-db.sh, called from .envrc with a find-newer
        # guard so they only run when sources change — not on every
        # `nix develop -c` invocation (e.g. CI).
      };
    };
}
