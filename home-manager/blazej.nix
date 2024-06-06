{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ ];

  home = {
    username = "blazej";
    homeDirectory = "/home/blazej";

    file = {
      ".config/dunst" = {
        source = config.lib.file.mkOutOfStoreSymlink ../dotfiles/dunst;
      };
      ".config/hypr" = {
        source = config.lib.file.mkOutOfStoreSymlink ../dotfiles/hypr;
      };
      ".config/rofi" = {
        source = config.lib.file.mkOutOfStoreSymlink ../dotfiles/rofi;
      };
      ".config/waybar" = {
        source = config.lib.file.mkOutOfStoreSymlink ../dotfiles/waybar;
      };
    };

    # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
    stateVersion = "24.05";
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
    git = {
      enable = true;
      userName = "Błażej Sowa";
      userEmail = "bsowa123@gmail.com";
    };
    home-manager.enable = true;
  };

  # Nicely reload system units when changing configs
  systemd.user.startServices = "sd-switch";
}
