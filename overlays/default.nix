{ inputs, ... }: {
  additions = final: _prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    flameshot = prev.flameshot.overrideAttrs (old: {
      src = prev.fetchFromGitHub {
        owner = "flameshot-org";
        repo = "flameshot";
        rev = "a1dda59108d420a26d603a40d1c7d25e4114d748";
        sha256 = "sha256-uISujW2Bqv07MpxCj9sbms//cJQJNRI+uJmkUphl1ds=";
      };
      cmakeFlags = old.cmakeFlags
        ++ [ (prev.lib.cmakeBool "USE_WAYLAND_GRIM" true) ];
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
    hyprland-git = inputs.hyprland.packages.${final.system};
  };
}
