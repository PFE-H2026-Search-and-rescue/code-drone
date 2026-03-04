{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";  # IMPORTANT!!!
  };
  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
        # Build colcon-uv from GitHub source
        colcon-uv = pkgs.python3Packages.buildPythonPackage rec {
          pname = "colcon-uv";
          version = "0.2.0";
          format = "pyproject";
          
          src = pkgs.fetchFromGitHub {
            owner = "nzlz";
            repo = "colcon-uv";
            rev = "v${version}";
            hash = "sha256-8U+7crw/86Qq4ztyC2s1aceLYLYukj4chqv/NyA/PGg=";
            fetchSubmodules = false;
          };
          
          sourceRoot = "${src.name}/colcon_uv";
          
          nativeBuildInputs = with pkgs.python3Packages; [
            setuptools
            hatchling
          ];
          
          propagatedBuildInputs = with pkgs.python3Packages; [
            colcon-core
          ];
          
          doCheck = false;
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "UAV-ROS-Development-Shell";
          packages = with pkgs;[
            python312
            uv
            colcon
            colcon-uv
            # ... other non-ROS packages
            (with rosPackages.humble; buildEnv {
              paths = [
                ros-core
                nav-msgs
                fastrtps
                # ... other ROS packages
              ];
            })
          ];
        };
      });
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
