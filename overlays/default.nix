{ inputs, ... }: {
  additions = final: prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    waybar = inputs.waybar-master.packages.${final.system}.waybar;
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
