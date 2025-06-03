{
  description = "Fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives.";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    coal-src = {
      url = "github:coal-library/coal";
      flake = false;
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
          apps.default = {
            type = "app";
            program = pkgs.python3.withPackages (_: [ self'.packages.default ]);
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
          packages = {
            default = self'.packages.pinocchio;
            coal = pkgs.coal.overrideAttrs (super: {
              src = inputs.coal-src;
              cmakeFlags = super.cmakeFlags ++ [ "-DCOAL_DISABLE_HPP_FCL_WARNINGS=ON" ];
            });
            pinocchio =
              (pkgs.pinocchio.overrideAttrs (super: {
                src = pkgs.lib.fileset.toSource {
                  root = ./.;
                  fileset = pkgs.lib.fileset.unions [
                    ./benchmark
                    ./bindings
                    ./CMakeLists.txt
                    ./doc
                    ./examples
                    ./include
                    ./models
                    ./package.xml
                    ./sources.cmake
                    ./src
                    ./unittest
                    ./utils
                  ];
                };
              })).override
                { inherit (self'.packages) coal; };
          };
        };
    };
}
