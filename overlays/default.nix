{ inputs, ... }: {
  additions = final: prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    hyprland = inputs.hyprland.packages.${final.system}.hyprland;
    xdg-desktop-portal-hyprland =
      inputs.hyprland.packages.${final.system}.xdg-desktop-portal-hyprland;

    kdePackages = prev.kdePackages.overrideScope (kfinal: kprev: {
      dolphin = kprev.dolphin.overrideAttrs (oldAttrs: {
        nativeBuildInputs = (oldAttrs.nativeBuildInputs or [ ])
          ++ [ prev.makeWrapper ];
        postInstall = (oldAttrs.postInstall or "") + ''
          wrapProgram $out/bin/dolphin \
              --set XDG_CONFIG_DIRS "${prev.libsForQt5.kservice}/etc/xdg:$XDG_CONFIG_DIRS" \
              --run "${kprev.kservice}/bin/kbuildsycoca6 --noincremental ${prev.libsForQt5.kservice}/etc/xdg/menus/applications.menu"
        '';
      });
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
