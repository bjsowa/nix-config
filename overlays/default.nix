{ inputs, ... }: {
  additions = final: prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    xwayland = inputs.nixpkgs-unstable.legacyPackages.${final.system}.xwayland;

    mymesa = inputs.nixpkgs-master.legacyPackages.${final.system}.mesa.overrideAttrs (old: {
      src = prev.fetchFromGitLab {
        domain = "gitlab.freedesktop.org";
        owner = "mesa";
        repo = "mesa";
        rev = "b74a6e05bd6d9bcbe1367695cb8c7a8c61f93ba8";
        hash = "sha256-JVDjZ4f6KJqb3eV3fekdbZdKauep+j0fa+SKv08iwyc=";
      };

      patches = [
        ./opencl.patch
        ./system-gbm.patch
      ];
    });
  };

  unstable-packages = final: _prev: {
    unstable = import inputs.nixpkgs-unstable {
      system = final.system;
      config.allowUnfree = true;
    };
    master = import inputs.nixpkgs-master {
      system = final.system;
      config.allowUnfree = true;
    };
    old = import inputs.nixpkgs-old {
      system = final.system;
      config.allowUnfree = true;
    };
  };
}
