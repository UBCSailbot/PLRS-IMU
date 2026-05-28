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
      unity-src = pkgs.fetchFromGitHub {
        owner = "ThrowTheSwitch";
        repo = "Unity";
        rev = "v2.6.1";
        hash = "sha256-g0ubq7RxGQmL1R6vz9RIGJpVWYsgrZhsTWSrL1ySEug=";
      };
    in
    {
      devShells.${system}.default = pkgs.mkShell {
        packages = with pkgs; [
          clang-tools
          jq
          platformio
          python3
        ];
        shellHook = ''
          # Generate compile databases for both environments.
          # pico build may download the ARM toolchain on first run (~100 MB, cached
          # in ~/.platformio afterward). Failures are non-fatal so a cold machine
          # without network still gets at least the native database.
          pio run -e native -t compiledb 2>/dev/null || true
          pio run -e pico   -t compiledb 2>/dev/null || true

          # Merge both databases into the project root so clangd gets one view
          # covering native test files (Unity flags) and firmware files (Arduino
          # headers, ARM defines). clangd searches upward for compile_commands.json
          # and uses the nearest match per translation unit.
          native=".pio/build/native/compile_commands.json"
          pico=".pio/build/pico/compile_commands.json"
          if [ -f "$native" ] && [ -f "$pico" ]; then
            jq -s '.[0] + .[1]' "$native" "$pico" > compile_commands.json
          elif [ -f "$pico" ]; then
            cp "$pico" compile_commands.json
          elif [ -f "$native" ]; then
            cp "$native" compile_commands.json
          fi

          cat > .clangd <<'EOF'
Index:
  Background: Build
CompileFlags:
  CompilationDatabase: .
EOF
        '';
      };
    };
}
