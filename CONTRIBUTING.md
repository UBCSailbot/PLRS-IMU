# Contributing

## Coding style

Formatting and linting are enforced by CI. Before pushing, run `make format`
(formats all C++, Nix, and Python in one shot), or individually:

- **C++ (`lib/`, `src/`, `test/`):** `clang-format -i <file>` (style in `.clang-format`; LLVM base, 80-column limit)
- **Nix files:** `make nix-format`
- **Python (`sim/`):** `make sim-format` (ruff format + lint)

Naming conventions: types and enum members in `PascalCase`, free functions and
methods in `snake_case`, constants in `UPPER_SNAKE_CASE`, private members with a
leading underscore (`_state`, `_sum`).

Design rules:

- `ByteSpan` (`std::span<const uint8_t>`) instead of `(ptr, len)` pairs.
- Outputs are returned, not written through out-parameters.
- `std::expected<T, E>` when failure has a meaningful reason, `std::optional`
  when it doesn't.
- `std::bit_cast` for type punning — no pointer casts, no `memcpy`.
- Named constants for all magic numbers.
- Statically allocate all memory; no `malloc`.
- `uint8_t` for wire bytes, not `char`. `char` signedness is
  implementation-defined (signed on x86, unsigned on ARM/RP2040); `0xFA` as a
  `signed char` silently becomes `-6`. `uint8_t` is unambiguously unsigned and
  signals raw protocol data, not text.
