{
  description = "A very basic flake";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  # Everything your flake provides
  outputs = { self, nixpkgs, flake-utils }: flake-utils.lib.eachDefaultSystem (system:
    let
      # This combined with the flake-utils package abstracts what architecture you are building for
      pkgs = import nixpkgs { 
        inherit system;
        config.allowUnfree = true;
      };

      # Shrimple dev shell to allow for local debug
      devShell = pkgs.mkShell {
        packages = with pkgs; [
          cmake
          ninja
          gcc-arm-embedded
          teensy-loader-cli
          teensy-udev-rules
        ];

        shellHook = ''
          echo "Use flash-teensy to upload a built firmware hex"
          alias flash-teensy='teensy-loader-cli --mcu=TEENSY41 -w -r ./build/src/firmware.hex'
        '';
      };

    in {
      devShells.default = devShell;
    }
  );
}
