#!/usr/bin/env bash
# Generate the merged compile_commands.json that clangd reads.
#
# Sourced from flake.nix shellHook so it runs every time the devshell is
# entered. Lives as a standalone file because the bash heredoc for .clangd
# doesn't survive nix string quoting + nixfmt cleanly inside flake.nix.
#
# `pio compiledb` overwrites the project-root compile_commands.json on each
# run, so we capture each env's output before invoking the next. The pico
# build may download the ARM toolchain on first run (~100 MB, cached
# afterward). Failures are non-fatal so a cold machine without network still
# gets at least the native database.

set -u

mkdir -p .pio
native=".pio/compile_commands.native.json"
pico=".pio/compile_commands.pico.json"
rm -f "$native" "$pico" compile_commands.json

# Pre-install lib_deps so their include paths appear in the compile
# databases. pio run -t compiledb also installs them, but doing it here
# makes failures visible independently of the compiledb step.
pio pkg install -e native 2>/dev/null || true
pio pkg install -e pico 2>/dev/null || true

pio run -e native -t compiledb 2>/dev/null || true
[ -f compile_commands.json ] && mv compile_commands.json "$native"

pio run -e pico -t compiledb 2>/dev/null || true
[ -f compile_commands.json ] && mv compile_commands.json "$pico"

# `pio compiledb` only emits an entry for src/main.cpp; test files need the
# Unity headers and PIO_UNIT_TESTING define which only appear in the actual
# test build command. Capture those by running the test build in verbose
# mode (without executing tests) and parsing the g++ lines that compile
# each test_main.cpp. Test .o files are removed first so PIO emits compile
# commands even on cached rebuilds.
if [ -f "$native" ]; then
  rm -f .pio/build/native/test/*/test_main.o
  test_log=$(mktemp)
  pio test -e native --without-uploading --without-testing -vvv \
    > "$test_log" 2>&1 || true
  test_entries=$(grep -E '^g\+\+ .* -c .* test/.*test_main\.cpp$' "$test_log" \
    | while read -r cmd; do
        file=$(printf '%s' "$cmd" | awk '{print $NF}')
        output=$(printf '%s' "$cmd" \
          | grep -oE -- '-o [^ ]+' \
          | head -1 \
          | awk '{print $2}')
        jq -n --arg dir "$PWD" \
              --arg file "$file" \
              --arg cmd "$cmd" \
              --arg output "$output" \
          '{directory: $dir, file: $file, command: $cmd, output: $output}'
      done | jq -s '.')
  rm -f "$test_log"
  if [ -n "$test_entries" ] && [ "$test_entries" != "[]" ]; then
    jq --argjson new "$test_entries" '. + $new' "$native" > "$native.tmp"
    mv "$native.tmp" "$native"
  fi
fi

# Merge both databases into the project root so clangd gets one view
# covering native test files (Unity flags) and firmware files (Arduino
# headers, ARM defines). clangd uses the first matching entry per TU, so
# deduplicate by file: keep native entries only for files absent from the
# pico DB (test files), and keep all pico entries (src/main.cpp + framework
# files). This prevents clangd from picking the host-g++ entry for
# src/main.cpp, which lacks Arduino/FreeRTOS headers.
if [ -f "$native" ] && [ -f "$pico" ]; then
  jq -s '
    (.[1] | map(.file)) as $pico_files |
    (.[0] | map(select(.file as $f | ($pico_files | index($f)) == null)))
    + .[1]
  ' "$native" "$pico" > compile_commands.json
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
