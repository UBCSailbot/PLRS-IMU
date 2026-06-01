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
          cmake
          jq
          ninja
          nixfmt
          platformio
          python3
          ruff
          uv
        ];
        # PyPI wheels (numpy and the rest of the SciPy stack) link against
        # libstdc++ at the host's standard path, which on Nix only lives in
        # the store. Expose the needed libs via LD_LIBRARY_PATH so import
        # works inside uv-managed venvs. (FHS is the alternative, but
        # `nix develop -c` doesn't enter its namespace.)
        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [
          pkgs.stdenv.cc.cc.lib
          pkgs.zlib
        ];
        # Heavy bash logic lives in scripts/setup-compile-db.sh; keeping
        # the shellHook a single line means nixfmt has nothing inside `''`
        # that it can mangle the indentation of.
        shellHook = ''
          bash ${./scripts/setup-compile-db.sh}
        '';
      };
    };
}
