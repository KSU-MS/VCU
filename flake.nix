{
  description = "KSU Motorsports VCU (Teensy 4.1) development environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        python = pkgs.python3.withPackages (ps: with ps; [
          python-can # host-side CAN tooling (OTA flasher, SIL harness)
          cantools # DBC encode/decode
          pytest
        ]);
      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            # nixpkgs' platformio is wrapped in an FHS env, so the prebuilt
            # toolchains PlatformIO downloads into ~/.platformio run on NixOS
            platformio
            teensy-loader-cli
            can-utils
            python
            clang-tools
          ];

          # PlatformIO's prebuilt arm-none-eabi-gcc (15.x) links against
          # libzstd, which the platformio FHS env doesn't provide
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.zstd ];

          shellHook = ''
            echo "VCU dev shell: pio run | pio test -e native | candump"
          '';
        };
      });
}
