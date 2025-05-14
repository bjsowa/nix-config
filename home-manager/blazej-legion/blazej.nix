{ inputs, outputs, config, lib, pkgs, ... }:
let
  iconThemePackage = (pkgs.catppuccin-papirus-folders.override {
    accent = "blue";
    flavor = "frappe";
  });
in {
  imports = [ inputs.stylix.homeManagerModules.stylix ];

  home = {
    username = "blazej";
    homeDirectory = "/home/${config.home.username}";

    packages = [
      iconThemePackage # Needs to be added for qt theming to work
    ];
    pointerCursor = { size = lib.mkForce 24; };

    # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
    stateVersion = "24.05";
  };

  dconf.settings = {
    "org/gnome/desktop/interface" = {
      color-scheme = lib.mkForce "prefer-dark";
    };
  };

  gtk = {
    enable = true;
    gtk3 = { extraConfig.gtk-application-prefer-dark-theme = true; };
    gtk4 = { extraConfig.gtk-application-prefer-dark-theme = true; };
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
      lfs.enable = true;
      userName = "Błażej Sowa";
      userEmail = "bsowa123@gmail.com";
      extraConfig = {
        url."ssh://git@github.com/" = {
          pushInsteadOf = "https://github.com/";
        };
      };
    };
    home-manager.enable = true;
    hyprlock = {
      enable = true;
      package = null;
      extraConfig = ''
        source = ~/.config/hypr/hyprlock-custom.conf
      '';
    };
    pyenv = {
      enable = true;
      enableBashIntegration = true;
    };
    rofi = {
      enable = true;
      package = pkgs.rofi-wayland;
    };
    waybar = {
      enable = true;
      package = pkgs.waybar;
      systemd = {
        enable = true;
        target = "hyprland-session.target";
      };
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

  services = {
    dunst = { enable = true; };
    hyprpaper = { enable = true; };
    mpris-proxy.enable = true;
  };

  stylix = {
    enable = true;
    base16Scheme = "${pkgs.base16-schemes}/share/themes/catppuccin-frappe.yaml";
    cursor = {
      name = "catppuccin-frappe-blue-cursors";
      package = pkgs.catppuccin-cursors.frappeBlue;
      size = 24;
    };
    iconTheme = {
      dark = "Papirus-Dark";
      package = iconThemePackage;
    };
    image = pkgs.fetchurl {
      url =
        "https://www.pixelstalk.net/wp-content/uploads/2016/05/Epic-Anime-Awesome-Wallpapers.jpg";
      sha256 = "enQo3wqhgf0FEPHj2coOCvo7DuZv+x5rL/WIo4qPI50=";
    };
    polarity = "dark";
    targets = {
      dunst.enable = false;
      waybar.enable = false;
    };
  };

  systemd.user = {
    # Nicely reload system units when changing configs
    startServices = "sd-switch";

    services = {
      polkit-kde-authentication-agent = {
        Unit = {
          Description = "KDE Polkit authentication agent";
          Documentation = "https://gitlab.freedesktop.org/polkit/polkit/";
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

        Install.WantedBy = [ "hyprland-session.target" ];
      };
    };
  };

  wayland.windowManager.hyprland = {
    enable = true;

    # Use packages defined in NixOS module
    package = null;
    portalPackage = null;

    # source our custom config
    extraConfig = ''
      source = ~/.config/hypr/hyprland-custom.conf
    '';
  };

  xdg = let dotfiles_path = "${config.home.homeDirectory}/nix-config/dotfiles";
  in {
    configFile = {
      "dunst/dunstrc".source =
        config.lib.file.mkOutOfStoreSymlink "${dotfiles_path}/dunst/dunstrc";
      "hypr/hyprland-custom.conf".source = config.lib.file.mkOutOfStoreSymlink
        "${dotfiles_path}/hypr/hyprland.conf";
      "hypr/hyprlock-custom.conf".source = config.lib.file.mkOutOfStoreSymlink
        "${dotfiles_path}/hypr/hyprlock.conf";
      "hypr/hyprpaper.conf".source = config.lib.file.mkOutOfStoreSymlink
        "${dotfiles_path}/hypr/hyprpaper.conf";
      "hypr/pyprland.toml".source = config.lib.file.mkOutOfStoreSymlink
        "${dotfiles_path}/hypr/pyprland.toml";
      "kdeglobals".source =
        config.lib.file.mkOutOfStoreSymlink "${dotfiles_path}/kdeglobals";
      "konsolerc".source = (pkgs.formats.ini { }).generate "konsolerc" {
        "Desktop Entry".DefaultProfile = "Default.profile";
      };
      "waybar".source =
        config.lib.file.mkOutOfStoreSymlink "${dotfiles_path}/waybar";
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
      "datafiles".source = config.lib.file.mkOutOfStoreSymlink
        "${config.home.homeDirectory}/nix-config/datafiles";
      "dunst/scripts".source = "${pkgs.my-nixos-scripts}/dunst";
      "hypr/scripts".source = "${pkgs.my-nixos-scripts}/hypr";
      "rofi/themes/config-custom.rasi".source =
        config.lib.file.mkOutOfStoreSymlink "${dotfiles_path}/rofi/config.rasi";
      "rofi/themes/custom.rasi".text = lib.mkAfter ''
        @import "config-custom"
      '';
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
