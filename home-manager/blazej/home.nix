{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ inputs.impermanence.nixosModules.home-manager.impermanence ];

  home = {
    username = "blazej";
    homeDirectory = "/home/blazej";

    file = {
      ".config/dunst".source = ../../dotfiles/dunst;
      ".config/hypr".source = ../../dotfiles/hypr;
      ".config/rofi".source = ../../dotfiles/rofi;
      ".config/waybar".source = ../../dotfiles/waybar;
    };

    packages = with pkgs; [
      (catppuccin-kvantum.override {
        accent = "Blue";
        variant = "Macchiato";
      })
      libsForQt5.qtstyleplugin-kvantum
      libsForQt5.qt5ct
      papirus-folders
    ];

    persistence."/persist/home/blazej" = {
      allowOther = true;
      directories = [
        "nix-config"
        "Downloads"
        "Music"
        "Pictures"
        "Documents"
        "Videos"
        ".gnupg"
        ".ssh"

        ".config/Code"
      ];
    };

    pointerCursor = {
      gtk.enable = true;
      name = "Catppuccin-Macchiato-Dark-Cursors";
      package = pkgs.catppuccin-cursors.macchiatoDark;
      size = 16;
    };

    # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
    stateVersion = "24.05";
  };

  gtk = {
    enable = true;
    theme = {
      name = "Catppuccin-Macchiato-Standard-Blue-Dark";
      package = pkgs.catppuccin-gtk.override {
        accents = [ "blue" ];
        size = "standard";
        variant = "macchiato";
      };
    };
    iconTheme = {
      name = "Papirus-Dark";
      package = pkgs.catppuccin-papirus-folders.override {
        flavor = "macchiato";
        accent = "blue";
      };
    };
    cursorTheme = {
      name = "Catppuccin-Macchiato-Dark-Cursors";
      package = pkgs.catppuccin-cursors.macchiatoDark;
    };
    gtk3 = { extraConfig.gtk-application-prefer-dark-theme = true; };
  };

  nixpkgs = {
    overlays = [
      outputs.overlays.additions
      outputs.overlays.modifications
      outputs.overlays.unstable-packages
    ];

    config = { allowUnfree = true; };
  };

  programs = {
    bash.enable = true;
    git = {
      enable = true;
      userName = "Błażej Sowa";
      userEmail = "bsowa123@gmail.com";
    };
    home-manager.enable = true;
  };

  qt = {
    enable = true;
    platformTheme.name = "qtct";
    style.name = "kvantum";
  };

  # Nicely reload system units when changing configs
  systemd.user.startServices = "sd-switch";

  xdg.configFile."Kvantum/kvantum.kvconfig".source =
    (pkgs.formats.ini { }).generate "kvantum.kvconfig" {
      General.theme = "Catppuccin-Macchiato-Blue";
    };
}
