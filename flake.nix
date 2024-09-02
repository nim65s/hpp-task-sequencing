{
  description = "Bindings between Numpy and Eigen using Boost.Python";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:gepetto/nixpkgs";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ... }:
        {
          packages = {
            default = self'.packages.hpp-task-sequencing;
            hpp-task-sequencing = pkgs.callPackage ./default.nix { };
          };
        };
    };
}
