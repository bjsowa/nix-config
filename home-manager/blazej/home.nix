{ inputs, outputs, lib, config, pkgs, ... }: {
  home = {
    username = "blazej";
    homeDirectory = "/home/${config.home.username}";

    packages = with pkgs; [
      (catppuccin-kvantum.override {
        accent = "Blue";
        variant = "Macchiato";
      })
      libsForQt5.qtstyleplugin-kvantum
      libsForQt5.qt5ct
      papirus-folders
    ];

    pointerCursor = {
      gtk.enable = true;
      name = "catppuccin-macchiato-dark-cursors";
      package = pkgs.catppuccin-cursors.macchiatoDark;
      size = 24;
    };

    # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
    stateVersion = "24.05";
  };

  dconf.settings = {
    "org/gnome/desktop/interface" = { color-scheme = "prefer-dark"; };
  };

  gtk = {
    enable = true;
    theme = {
      name = "catppuccin-macchiato-blue-standard";
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
      name = "catppuccin-macchiato-dark-cursors";
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
    direnv = {
      enable = true;
      enableBashIntegration = true;
      nix-direnv.enable = true;
    };
    git = {
      enable = true;
      userName = "Błażej Sowa";
      userEmail = "bsowa123@gmail.com";
    };
    home-manager.enable = true;
    pyenv = {
      enable = true;
      enableBashIntegration = true;
    };
    zsh = {
      enable = true;
      enableCompletion = true;
      autosuggestion.enable = true;
      syntaxHighlighting.enable = true;
      history = {
        size = 10000;
        path = "${config.xdg.dataHome}/zsh/history";
      };
      oh-my-zsh = {
        enable = true;
        custom =
          "${config.home.homeDirectory}/.local/share/datafiles/oh-my-zsh";
        theme = "agnoster-custom";
        plugins = [
          "aliases"
          "common-aliases"
          "colored-man-pages"
          "copybuffer"
          "dircycle"
          "git"
          "last-working-dir"
          "magic-enter"
          "nmap"
          "safe-paste"
          "scd"
          "sudo"
          "timer"
          "transfer"
          "zbell"
        ];
      };
      envExtra = ''
        export DEFAULT_USER=${config.home.username}
      '';
      shellAliases = let COLCON_COMMON_ARGS = "--symlink-install";
      in {
        cc-build =
          "colcon build ${COLCON_COMMON_ARGS} --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON";
        cc-buildpackage =
          "colcon build ${COLCON_COMMON_ARGS} --event-handlers console_direct+ --cmake-clean-cache --packages-select";
        cc-buildpackage-verbose =
          "VERBOSE=1 colcon build ${COLCON_COMMON_ARGS} --event-handlers console_direct+ --cmake-clean-cache --packages-select";
        cc-buildupto =
          "colcon build ${COLCON_COMMON_ARGS} --event-handlers console_direct+ --cmake-clean-cache --packages-up-to";
        cc-test = "colcon test";
        cc-testpackage =
          "colcon test --event-handlers console_direct+ --packages-select";
        cc-testpackage-verbose =
          "VERBOSE=1 colcon test --event-handlers console_direct+ --packages-select";
        cc-clean = "colcon build ${COLCON_COMMON_ARGS} --cmake-target clean";
        cc-cleanpackage =
          "colcon build ${COLCON_COMMON_ARGS} --cmake-target clean --packages-select";
      };
    };
  };

  qt = {
    enable = true;
    platformTheme.name = "qtct";
    style.name = "kvantum";
  };

  services = { mpris-proxy.enable = true; };

  systemd.user = {
    # Nicely reload system units when changing configs
    startServices = "sd-switch";

    services = {
      polkit-kde-authentication-agent = {
        Unit = {
          Description = "KDE Polkit authentication agent";
          Documentation = "https://gitlab.freedesktop.org/polkit/polkit/";
          After = [ "graphical-session-pre.target" ];
          PartOf = [ "graphical-session.target" ];
        };

        Service = {
          Type = "simple";
          ExecStart =
            "${pkgs.libsForQt5.polkit-kde-agent}/libexec/polkit-kde-authentication-agent-1";
          Restart = "always";
          RestartSec = 1;
          TimeoutStopSec = 10;
        };

        Install.WantedBy = [ "graphical-session.target" ];
      };
    };
  };

  wayland.windowManager.hyprland = {
    enable = true;
    extraConfig = builtins.readFile ../../dotfiles/hypr/hyprland.conf;
    # package = pkgs.hyprland-git.hyprland;
  };

  xdg = {
    configFile = {
      "Kvantum/kvantum.kvconfig".source =
        (pkgs.formats.ini { }).generate "kvantum.kvconfig" {
          General.theme = "Catppuccin-Macchiato-Blue";
        };
      "dunst".source = ../../dotfiles/dunst;
      "hypr/pyprland.toml".source = ../../dotfiles/hypr/pyprland.toml;
      "konsolerc".source = (pkgs.formats.ini { }).generate "konsolerc" {
        "Desktop Entry".DefaultProfile = "Default.profile";
      };
      "qt5ct/qt5ct.conf".source = (pkgs.formats.ini { }).generate "qt5ct.conf" {
        Appearance.icon_theme = "Papirus-Dark";
      };
      "rofi".source = ../../dotfiles/rofi;
      "waybar".source = ../../dotfiles/waybar;
      "yabridgectl/config.toml".source =
        (pkgs.formats.toml { }).generate "config.toml" {
          plugin_dirs = [
            "${config.home.homeDirectory}/.wine/drive_c/Program Files/Common Files/VST3"
          ];
          vst2_location = "centralized";
          no_verify = false;
          blacklist = [ ];
        };
    };
    dataFile = {
      "datafiles".source = ../../datafiles;
      "dunst/scripts".source = "${pkgs.my-nixos-scripts}/dunst";
      "hypr/scripts".source = "${pkgs.my-nixos-scripts}/hypr";
      "waybar/scripts".source = "${pkgs.my-nixos-scripts}/waybar";
      "konsole/Default.profile".source =
        (pkgs.formats.ini { }).generate "Default.profile" {
          Appearance.Font = "DejaVu Sans Mono,12,-1,5,50,0,0,0,0,0";
          General = {
            Name = "Default";
            Parent = "FALLBACK/";
          };
          Scrolling.HistorySize = 50000;
        };
    };

    mimeApps = { enable = true; };
  };
}
