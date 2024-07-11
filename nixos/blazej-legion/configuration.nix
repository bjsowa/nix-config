{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    ./specialisations/default.nix
    inputs.impermanence.nixosModules.impermanence
    inputs.home-manager.nixosModules.home-manager
    inputs.nixos-hardware.nixosModules.common-cpu-amd
    inputs.nixos-hardware.nixosModules.common-cpu-amd-pstate
    inputs.nixos-hardware.nixosModules.common-gpu-amd
    inputs.nixos-hardware.nixosModules.common-pc-laptop
    inputs.nixos-hardware.nixosModules.common-pc-laptop-ssd
    outputs.nixosModules.schroot
  ];

  boot = {
    binfmt.emulatedSystems = [ "aarch64-linux" ];

    blacklistedKernelModules = [ "ideapad_laptop" ];

    kernel.sysctl."kernel.sysrq" = 1;

    kernelPackages = lib.mkDefault pkgs.master.linuxPackages_6_6;

    kernelParams = [ "acpi_osi=Linux" "amdgpu.sg_display=0" ];

    loader = {
      efi.canTouchEfiVariables = true;
      grub = {
        enable = true;
        device = "nodev";
        efiSupport = true;
      };
    };

    initrd.luks.devices.cryptroot.device =
      "/dev/disk/by-uuid/1200359f-6591-46d5-8de4-85bea1ab9a59";
    initrd.postDeviceCommands = lib.mkAfter ''
      mkdir /btrfs_tmp
      mount /dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e /btrfs_tmp
      if [[ -e /btrfs_tmp/root ]]; then
        mkdir -p /btrfs_tmp/old_roots
        timestamp=$(date --date="@$(stat -c %Y /btrfs_tmp/root)" "+%Y-%m-%-d_%H:%M:%S")
        mv /btrfs_tmp/root "/btrfs_tmp/old_roots/$timestamp"
      fi

      delete_subvolume_recursively() {
        IFS=$'\n'
        for i in $(btrfs subvolume list -o "$1" | cut -f 9- -d ' '); do
          delete_subvolume_recursively "/btrfs_tmp/$i"
        done
        btrfs subvolume delete "$1"
      }

      for i in $(find /btrfs_tmp/old_roots/ -maxdepth 1 -mtime +30); do
        delete_subvolume_recursively "$i"
      done

      btrfs subvolume create /btrfs_tmp/root
      umount /btrfs_tmp
    '';
  };

  console = {
    earlySetup = true;
    font = "${pkgs.terminus_font}/share/consolefonts/ter-v32n.psf.gz";
    keyMap = "pl";
  };

  environment = {
    persistence."/persist/system" = {
      hideMounts = true;
      directories = [
        "/var/cache"
        "/var/log"
        "/var/lib/bluetooth"
        "/var/lib/nixos"
        "/var/lib/systemd/coredump"
        "/etc/NetworkManager/system-connections"
        "/srv/chroot"
      ];
      files = [ "/etc/machine-id" ];
      users.blazej = {
        directories = [
          "Documents"
          "Downloads"
          "Games"
          "Music"
          "Pictures"
          ".Private"
          "Videos"

          "nix-config"
          "praca"

          ".cache"
          ".ecryptfs"
          ".gitkraken"
          ".gnupg"
          ".platformio"
          ".ssh"
          ".thunderbird"
          ".wine"
          ".vscode"
          ".vst3"

          ".config/Caprine"
          ".config/Code"
          ".config/Element"
          ".config/GitKraken"
          ".config/REAPER"
          ".config/Slack"
          ".config/vivaldi"

          ".local/share/direnv"
          ".local/share/dolphin"
          ".local/share/konsole"
          ".local/share/lutris"
          ".local/share/pyenv"
          ".local/share/Steam"
          ".local/share/zsh"
        ];
        files = [
          ".bash_history"
          ".config/bloom"
          ".config/hypr/monitors.conf"
          ".scdhistory"
        ];
      };
    };

    sessionVariables = { NIXOS_OZONE_WL = "1"; };
  };

  environment = {
    etc = {
      "schroot/schroot.conf".source =
        (pkgs.formats.ini { }).generate "schroot.conf" {
          "focal" = {
            type = "directory";
            description = "Ubuntu 20.04 Focal Fossa";
            directory = "/srv/chroot/focal";
            users = "blazej";
            root-users = "blazej";
            personality = "linux";
            preserve-environment = false;
            profile = "my-profile";
            shell = "/bin/zsh";
          };
          "jammy" = {
            type = "directory";
            description = "Ubuntu 22.04 Jammy Jellyfish";
            directory = "/srv/chroot/jammy";
            users = "blazej";
            root-users = "blazej";
            personality = "linux";
            preserve-environment = false;
            profile = "my-profile";
            shell = "/bin/zsh";
          };
          "noble" = {
            type = "directory";
            description = "Ubuntu 24.04 Noble";
            directory = "/srv/chroot/noble";
            users = "blazej";
            root-users = "blazej";
            personality = "linux";
            preserve-environment = false;
            profile = "my-profile";
            shell = "/bin/zsh";
          };
        };
      "schroot/my-profile".source = ../../dotfiles/schroot/my-profile;
    };

    systemPackages = with pkgs; [
      autoconf
      automake
      brightnessctl
      caprine-bin
      cmake
      cryptsetup
      debootstrap
      dmidecode
      dunst
      ecryptfs
      element-desktop
      flameshot
      font-manager
      gcc
      git
      gitkraken
      gnumake
      grim
      file
      htop
      keyutils
      lenovo-legion
      libsForQt5.ark
      libsForQt5.dolphin
      libsForQt5.ffmpegthumbs
      libsForQt5.gwenview
      libsForQt5.kdegraphics-thumbnailers
      libsForQt5.kio-admin
      libsForQt5.kio-extras
      libsForQt5.konsole
      libsForQt5.okular
      libtool
      light
      lm_sensors
      lshw
      lutris
      mattermost-desktop
      mpv
      ncdu
      ninja
      nix-output-monitor
      nmap
      nwg-displays
      pamixer
      pavucontrol
      playerctl
      pkg-config
      protonup
      pulseaudio
      pyprland
      python3
      qbittorrent-qt5
      qjackctl
      reaper
      rofi-wayland
      screen
      slack
      swaybg
      swaylock-effects
      thunderbird
      tor-browser
      waybar
      wdisplays
      wget
      wireshark
      vim
      wineWowPackages.staging
      winetricks
      vivaldi
      vivaldi-ffmpeg-codecs
      wl-clipboard
      vscode
      yabridge
      yabridgectl
      yarn
    ];
  };

  fileSystems."/persist".neededForBoot = true;

  fonts = {
    fontDir.enable = true;
    packages = with pkgs; [ nerdfonts font-awesome google-fonts ];
  };

  hardware = {
    bluetooth = {
      enable = true;
      powerOnBoot = true;
    };

    opengl = {
      enable = true;
      driSupport = true;
      driSupport32Bit = true;
    };
  };

  home-manager = {
    extraSpecialArgs = { inherit inputs outputs; };
    users = { blazej = import ../../home-manager/blazej/home.nix; };
  };

  i18n = {
    defaultLocale = "en_US.UTF-8";
    supportedLocales = [
      "C.UTF-8/UTF-8"
      "en_US.UTF-8/UTF-8"
      "pl_PL.UTF-8/UTF-8"
      "ja_JP.UTF-8/UTF-8"
    ];
  };

  networking = {
    hostName = "blazej-legion";
    networkmanager.enable = true;
    firewall.enable = false;
  };

  nixpkgs = {
    overlays = [
      outputs.overlays.additions
      outputs.overlays.modifications
      outputs.overlays.unstable-packages
    ];
    config = { allowUnfree = true; };
  };

  nix = let flakeInputs = lib.filterAttrs (_: lib.isType "flake") inputs;
  in {
    settings = {
      experimental-features = "nix-command flakes";
      # Opinionated: disable global registry
      flake-registry = "";
      # Workaround for https://github.com/NixOS/nix/issues/9574
      nix-path = config.nix.nixPath;

      trusted-users = [ "root" "blazej" ];
    };
    # Opinionated: disable channels
    channel.enable = false;

    # Opinionated: make flake registry and nix path match flake inputs
    registry = lib.mapAttrs (_: flake: { inherit flake; }) flakeInputs;
    nixPath = lib.mapAttrsToList (n: _: "${n}=flake:${n}") flakeInputs;
  };

  programs = {
    appimage = {
      enable = true;
      binfmt = true;
    };

    ecryptfs.enable = true;

    fuse.userAllowOther = true;

    # gamemode.enable = true;

    gnupg.agent = {
      enable = true;
      enableSSHSupport = true;
    };

    hyprland = {
      enable = true;
      xwayland.enable = true;
    };

    nix-ld = { enable = true; };

    schroot.enable = true;

    steam = {
      enable = true;
      gamescopeSession.enable = true;
    };

    wireshark.enable = true;

    zsh.enable = true;
  };

  security.pam.services.swaylock = { };
  security.polkit.enable = true;

  services = {
    avahi = {
      enable = true;
      nssmdns4 = true;
      openFirewall = true;
    };

    displayManager = {
      sddm = {
        enable = true;
        wayland.enable = true;
      };

      autoLogin = {
        enable = true;
        user = "blazej";
      };

      defaultSession = "hyprland";
    };

    fstrim.enable = true;

    logind.lidSwitch = "ignore";

    libinput.enable = true;

    locate.enable = true;

    openssh = {
      enable = true;
      settings = {
        PermitRootLogin = "no";
        PasswordAuthentication = false;
      };
    };

    pipewire = {
      enable = true;
      alsa.enable = true;
      alsa.support32Bit = true;
      pulse.enable = true;
      jack.enable = true;
    };

    # AMD has better battery life with PPD over TLP:
    # https://community.frame.work/t/responded-amd-7040-sleep-states/38101/13
    power-profiles-daemon.enable = true;

    printing = {
      enable = true;
      drivers = [ ];
      # browsing = true;
      # defaultShared = true;
    };

    # thermald = {
    #   enable = true;
    #   ignoreCpuidCheck = true;
    # };

    xserver = {
      enable = true;
      xkb = {
        layout = "pl";
        variant = "";
      };
    };

    udev.extraRules = ''
      # ST-Link V3MINIE
      ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="660", GROUP="plugdev", TAG+="uaccess"

      # Luxonis OAK cameras
      SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
    '';

    udev.packages = [ pkgs.openocd ];

    udisks2 = { enable = true; };

  };

  specialisation = {
    nvidia.configuration = {
      imports = [ ./specialisations/nvidia.nix ];
      disabledModules = [ ./specialisations/default.nix ];
    };
    rt-audio.configuration = {
      imports = [ ./specialisations/rt-audio.nix ];
      disabledModules = [ ./specialisations/default.nix ];
    };
  };

  sound.enable = true;

  # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
  system.stateVersion = "24.05";

  time.timeZone = "Europe/Warsaw";

  users = {
    groups = { plugdev = { }; };
    users = {
      blazej = {
        hashedPasswordFile = "/persist/passwords/blazej";
        isNormalUser = true;
        extraGroups =
          [ "audio" "dialout" "networkmanager" "plugdev" "wheel" "wireshark" ];
        shell = pkgs.zsh;
      };
    };
  };

  xdg.mime.defaultApplications = {
    "text/html" = "vivaldi-stable.desktop";
    "x-scheme-handler/http" = "vivaldi-stable.desktop";
    "x-scheme-handler/https" = "vivaldi-stable.desktop";
    "x-scheme-handler/about" = "vivaldi-stable.desktop";
    "x-scheme-handler/unknown" = "vivaldi-stable.desktop";
  };
}
