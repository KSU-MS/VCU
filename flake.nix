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
        lib = pkgs.lib;

        python = pkgs.python3.withPackages (ps: with ps; [
          python-can # host-side CAN tooling (OTA flasher, SIL harness)
          can-isotp # ISO-TP over any python-can interface
          udsoncan # UDS client for tools/ota/flash.py
          cantools # DBC encode/decode
          olefile # tools/schematic reads Altium files
          pytest
        ]);

        # Everything both the dev shell and the dev container get
        tools = with pkgs; [
          teensy-loader-cli
          can-utils
          python
          clang-tools
        ];

        # Libraries the prebuilt toolchains PlatformIO downloads into
        # ~/.platformio link against (arm-none-eabi-gcc 15 needs libzstd)
        toolchainLibs = with pkgs; [ zstd zlib stdenv.cc.cc.lib libusb1 ];
      in
      {
        devShells.default = pkgs.mkShell {
          # nixpkgs' platformio is wrapped in an FHS env so the downloaded
          # toolchains run on NixOS
          packages = [ pkgs.platformio ] ++ tools;

          # ...but that FHS env doesn't include libzstd
          LD_LIBRARY_PATH = lib.makeLibraryPath [ pkgs.zstd ];

          shellHook = ''
            echo "VCU dev shell: pio run | pio test -e native | candump"
          '';
        };

        # The same environment as a container image, for people without Nix.
        # See .devcontainer/. Inside a container the FHS-wrapped platformio
        # would need user namespaces, so this uses the plain platformio-core
        # and gives the downloaded toolchains a normal /lib64 loader instead
        # (what nix-ld does on NixOS).
        packages.devcontainer = pkgs.dockerTools.buildLayeredImage {
          name = "kms-vcu-dev";
          tag = "latest";

          contents = with pkgs; [
            platformio-core
            gcc # native tests and the SIL build
            bashInteractive
            coreutils
            findutils
            gnugrep
            gnused
            gawk
            gnutar
            gzip
            which
            git
            openssh
            iproute2 # vcan setup
            util-linux # unshare, for tools/sil/run_e2e.sh
            cacert
            glibc.bin # ldd/getconf, which VS Code's server probes for
            procps
            dockerTools.usrBinEnv
            dockerTools.fakeNss
          ] ++ tools;

          extraCommands = ''
            mkdir -p lib64 tmp root
            chmod 1777 tmp
            ln -s ${pkgs.glibc}/lib/ld-linux-x86-64.so.2 lib64/ld-linux-x86-64.so.2
          '';

          config = {
            Cmd = [ "bash" ];
            WorkingDir = "/workspaces";
            Env = [
              "LD_LIBRARY_PATH=${lib.makeLibraryPath toolchainLibs}"
              "SSL_CERT_FILE=${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt"
              "GIT_SSL_CAINFO=${pkgs.cacert}/etc/ssl/certs/ca-bundle.crt"
              "HOME=/root"
              # The bind-mounted repo is owned by the host user, and git
              # refuses it otherwise, which breaks pio-git-hash-gen
              "GIT_CONFIG_COUNT=1"
              "GIT_CONFIG_KEY_0=safe.directory"
              "GIT_CONFIG_VALUE_0=*"
              "PS1=vcu-dev:\\w$ "
            ];
          };
        };
      });
}
