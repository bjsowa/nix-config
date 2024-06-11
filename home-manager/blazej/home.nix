{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ inputs.impermanence.nixosModules.home-manager.impermanence ];

  home = {
    username = "blazej";
    homeDirectory = "/home/blazej";

    file.".local/share/konsole/Default.profile".text = ''
      [Appearance]
      Font=DejaVu Sans Mono,12,-1,5,50,0,0,0,0,0

      [General]
      Name=Default
      Parent=FALLBACK/
    '';

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
        "Documents"
        "Downloads"
        "Music"
        "Pictures"
        "Videos"
        "nix-config"

        ".cache"
        ".gnupg"
        ".ssh"
        ".vscode"

        ".config/Caprine"
        ".config/Code"
        ".config/Element"
        ".config/Slack"
        ".config/vivaldi"

        ".local/share/dolphin"
        ".local/share/konsole"
      ];
      files = [ ".bash_history" ];
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

  xdg = {
    configFile = {
      "Kvantum/kvantum.kvconfig".source =
        (pkgs.formats.ini { }).generate "kvantum.kvconfig" {
          General.theme = "Catppuccin-Macchiato-Blue";
        };
      "dunst".source = ../../dotfiles/dunst;
      "hypr".source = ../../dotfiles/hypr;
      "konsolerc".source = (pkgs.formats.ini { }).generate "konsolerc" {
        "Desktop Entry".DefaultProfile = "Default.profile";
      };
      "qt5ct/qt5ct.conf".source = (pkgs.formats.ini { }).generate "qt5ct.conf" {
        Appearance.icon_theme = "Papirus-Dark";
      };
      "rofi".source = ../../dotfiles/rofi;
      "waybar".source = ../../dotfiles/waybar;
    };
    dataFile = {
      "datafiles".source = ../../datafiles;
      "dunst/scripts".source = "${pkgs.my-nixos-scripts}/dunst";
      "hypr/scripts".source = "${pkgs.my-nixos-scripts}/hypr";
      "waybar/scripts".source = "${pkgs.my-nixos-scripts}/waybar";
    };
  };
}
