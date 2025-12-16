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
      pkgs = import nixpkgs { inherit system; };

      # Shrimple dev shell to allow for local debug
      devShell = pkgs.mkShell {
        packages = with pkgs; [
          cmake
          ninja
          gcc-arm-embedded
        ];

        shellHook = ''
          echo "mrow"
        '';
      };

    in {
      devShells.default = devShell;
    }
  );
}
