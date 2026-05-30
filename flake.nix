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
          # Generate compile databases for both environments. `pio compiledb`
          # overwrites the project-root compile_commands.json on each run, so
          # capture each env's output before invoking the next.
          # pico build may download the ARM toolchain on first run (~100 MB,
          # cached afterward). Failures are non-fatal so a cold machine without
          # network still gets at least the native database.
          mkdir -p .pio
          native=".pio/compile_commands.native.json"
          pico=".pio/compile_commands.pico.json"
          rm -f "$native" "$pico" compile_commands.json

          pio run -e native -t compiledb 2>/dev/null || true
          [ -f compile_commands.json ] && mv compile_commands.json "$native"

          pio run -e pico -t compiledb 2>/dev/null || true
          [ -f compile_commands.json ] && mv compile_commands.json "$pico"

          # `pio compiledb` only emits entries for src/, so test/*.cpp would be
          # invisible to clangd. Template entries from the native src/main.cpp
          # command (identical flags, only filenames differ).
          if [ -f "$native" ]; then
            template=$(jq '.[0]' "$native")
            if [ -n "$template" ] && [ "$template" != "null" ]; then
              test_entries=$(find test -name "*.cpp" -type f | while read -r f; do
                printf '%s\n' "$template" | jq --arg f "$f" '
                  .file = $f
                  | .command = (.command | sub("src/main\\.cpp"; $f))
                  | .output = ".pio/build/native/" + ($f | sub("\\.cpp$"; ".o"))
                '
              done | jq -s '.')
              jq --argjson new "$test_entries" '. + $new' "$native" > "$native.tmp"
              mv "$native.tmp" "$native"
            fi
          fi

          # Merge both databases into the project root so clangd gets one view
          # covering native test files (Unity flags) and firmware files (Arduino
          # headers, ARM defines). clangd uses the entry matching each TU.
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
