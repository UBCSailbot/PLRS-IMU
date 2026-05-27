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
          platformio
          python3
        ];
        shellHook = ''
          # Regenerate compile_commands.json so clangd sees current source files.
          pio run -e native -t compiledb 2>/dev/null

          # Derive the C++ standard from platformio.ini so .clangd never drifts.
          # Reads the [env:native] section; stops at the next section header.
          std=$(awk '/^\[env:native\]/{f=1;next} /^\[/{f=0} f' platformio.ini \
            | grep -oP '(?<=-std=)\S+' | head -1)

          cat > .clangd <<EOF
Index:
  Background: Build
CompileFlags:
  Compiler: $(which g++)
  Add: [-std=$std, -Ilib/mti_imu, -I${unity-src}/src]
  Remove: [-std=c++14, -std=c++17, -std=c++20, -std=gnu++14, -std=gnu++17, -std=gnu++20]
EOF
        '';
      };
    };
}
