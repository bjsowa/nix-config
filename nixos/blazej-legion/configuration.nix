{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    ./nvidia.nix
    ./schroot.nix
    inputs.impermanence.nixosModules.impermanence
    inputs.home-manager.nixosModules.home-manager
    inputs.nixos-hardware.nixosModules.common-cpu-amd
    inputs.nixos-hardware.nixosModules.common-cpu-amd-pstate
    inputs.nixos-hardware.nixosModules.common-gpu-amd
    inputs.nixos-hardware.nixosModules.common-pc-laptop
    inputs.nixos-hardware.nixosModules.common-pc-laptop-ssd
    inputs.catppuccin.nixosModules.catppuccin
  ];

  boot = {
    binfmt = {
      emulatedSystems = [ "aarch64-linux" "armv7l-linux" ];
      registrations = {
        "aarch64-linux" = { fixBinary = true; };
        "armv7l-linux" = { fixBinary = true; };
      };
      preferStaticEmulators = true;
    };

    blacklistedKernelModules = [ "ideapad_laptop" ];

    extraModulePackages = with config.boot.kernelPackages; [ v4l2loopback ];

    kernel.sysctl."kernel.sysrq" = 1;

    kernelPackages = lib.mkDefault pkgs.linuxPackages_6_12;

    kernelParams = [ "amdgpu.abmlevel=0" ];

    loader = {
      efi.canTouchEfiVariables = true;
      grub = {
        enable = true;
        device = "nodev";
        efiSupport = true;
        useOSProber = true;
      };
    };

    initrd.luks.devices.cryptroot.device =
      "/dev/disk/by-uuid/1200359f-6591-46d5-8de4-85bea1ab9a59";
    initrd.postDeviceCommands = lib.mkAfter ''
      mkdir /btrfs_tmp
      mount /dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e /btrfs_tmp
      if [[ -e /btrfs_tmp/root ]]; then
        timestamp=$(date --date="@$(stat -c %Y /btrfs_tmp/root)" "+%Y-%m-%d_%H:%M:%S")
        mv /btrfs_tmp/root "/btrfs_tmp/old_roots/$timestamp"
        rm -f /btrfs_tmp/old_roots/latest
        ln -s $timestamp /btrfs_tmp/old_roots/latest
      fi

      delete_subvolume_recursively() {
        IFS=$'\n'
        for i in $(btrfs subvolume list -o "$1" | cut -f 9- -d ' '); do
          delete_subvolume_recursively "/btrfs_tmp/$i"
        done
        btrfs subvolume delete "$1"
      }

      for i in $(find /btrfs_tmp/old_roots/ -maxdepth 1 -mtime +7); do
        delete_subvolume_recursively "$i"
      done

      btrfs subvolume create /btrfs_tmp/root
      umount /btrfs_tmp
    '';

    supportedFilesystems = { ntfs = true; };
  };

  catppuccin = {
    enable = true;
    flavor = "frappe";
    accent = "blue";
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
        "/var/lib/flatpak"
        "/var/lib/libvirt"
        "/var/lib/nixos"
        "/var/lib/systemd/coredump"
        "/etc/NetworkManager/system-connections"
        "/etc/cups"
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
          ".vscode-server"
          ".vst3"

          ".config/anytype"
          ".config/BetterDiscord"
          ".config/Caprine"
          ".config/Code"
          ".config/cosmic"
          ".config/discord"
          ".config/Element"
          ".config/GitKraken"
          ".config/hcloud"
          ".config/Mattermost"
          ".config/PrusaSlicer"
          ".config/rclone"
          ".config/REAPER"
          ".config/Slack"
          ".config/teamviewer"
          ".config/vivaldi"
          ".config/YouTube Music"

          ".local/share/direnv"
          ".local/share/dolphin"
          ".local/share/flatpak"
          ".local/share/keyrings"
          ".local/share/konsole"
          ".local/share/lutris"
          ".local/share/MikroTik"
          ".local/share/pyenv"
          ".local/share/Steam"
          ".local/share/zsh"

          ".local/state/nix"
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
    systemPackages = with pkgs; [
      anytype
      autoconf
      automake
      brightnessctl
      caprine
      clang-tools
      cmake
      cryptsetup
      dconf-editor
      debootstrap
      discord
      dmidecode
      dunst
      ecryptfs
      element-desktop
      exfatprogs
      file
      font-manager
      gcc
      gdb
      git
      gitkraken
      gnumake
      gparted
      grim
      hcloud
      htop
      hyprshot
      keyutils
      lenovo-legion
      kdePackages.ark
      kdePackages.dolphin
      kdePackages.ffmpegthumbs
      kdePackages.gwenview
      kdePackages.kcalc
      kdePackages.kdegraphics-thumbnailers
      kdePackages.kdenlive
      kdePackages.kio-admin
      kdePackages.kio-extras
      kdePackages.konsole
      kdePackages.okular
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
      nix-tree
      nodejs
      nurl
      nixd
      nixfmt-classic
      nmap
      nwg-displays
      pamixer
      pavucontrol
      pciutils
      playerctl
      pkg-config
      protonup
      prusa-slicer
      pulseaudio
      pyprland
      python3
      qbittorrent
      qjackctl
      reaper
      rclone
      screen
      sidequest
      simple-scan
      slack
      sshfs
      syncplay
      system-config-printer
      thunderbird
      tor-browser
      unrar
      usbutils
      wdisplays
      wget
      winbox4
      wireshark
      vim
      wineWowPackages.staging
      winetricks
      (vivaldi.override {
        proprietaryCodecs = true;
        enableWidevine = true;
      })
      wl-clipboard
      vscode
      xorg.xeyes
      xorg.xhost
      yabridge
      yabridgectl
      yarn
      youtube-music
      yt-dlp
    ];
  };

  fileSystems."/persist".neededForBoot = true;

  fonts = {
    fontDir.enable = true;
    packages = with pkgs;
      [ font-awesome google-fonts ] ++ builtins.filter lib.attrsets.isDerivation
      (builtins.attrValues pkgs.nerd-fonts);
  };

  hardware = {
    bluetooth = {
      enable = true;
      powerOnBoot = true;
    };

    graphics = {
      enable = true;
      enable32Bit = true;

      # package = pkgs.unstable.mesa;
      # package32 = pkgs.unstable.pkgsi686Linux.mesa;
    };

    sane = {
      brscan4 = {
        enable = true;
        netDevices = {
          fictionlab-brother = {
            model = "MFC-L2700DW";
            ip = "192.168.1.21";
          };
        };
      };
      brscan5.enable = true;
      enable = true;
    };
  };

  home-manager = {
    extraSpecialArgs = {
      inherit inputs outputs;
      # pkgs = pkgs // { formats = pkgs.unstable.formats; };
    };
    users = { blazej = import ../../home-manager/blazej-legion/blazej.nix; };
    useGlobalPkgs = true;
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

    wg-quick.interfaces = {
      wg-io = {
        autostart = false;
        address = [ "10.100.0.2/24" ];
        privateKeyFile = "/persist/secrets/wg-private";

        peers = [{
          publicKey = "io/aP205KKnDPV8GYWUbIfnodrjl4lwdcEFMhM9IlE4=";
          allowedIPs = [ "0.0.0.0/0" ];
          endpoint = "78.46.205.86:51820";
          persistentKeepalive = 25;
        }];
      };
    };
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
      flake-registry = "";
      trusted-users = lib.mkAfter [ "blazej" ];
      trusted-public-keys =
        lib.mkAfter [ "flnix:+tMGg+wtEYV7Fc8bEiZcCttdjqnsHbkNP/zc9AfEPNY=" ];
    };
    # package = pkgs.unstable.nix;

    distributedBuilds = true;

    buildMachines = [
      # flnix
      {
        system = "aarch64-linux";
        sshUser = "pi";
        hostName = "192.168.1.6";
        sshKey = "/persist/secrets/ssh_flnix";
        supportedFeatures = [ "kvm" ];
        maxJobs = 4;
        publicHostKey =
          "c3NoLWVkMjU1MTkgQUFBQUMzTnphQzFsWkRJMU5URTVBQUFBSUx1dllpMDU3bXhRU1BUODFTQUt0TDJsZkE2Y0xOQWtYOUM5dDE1NTVBcEkgcm9vdEBmbG5peAo=";
      }
    ];

    # Opinionated: disable channels
    channel.enable = false;

    # Make flake registry and nix path match flake inputs
    registry = lib.mapAttrs (_: flake: { inherit flake; }) flakeInputs;
    nixPath = lib.mapAttrsToList (n: _: "${n}=flake:${n}") flakeInputs;
  };

  programs = {
    adb.enable = true;

    alvr = {
      enable = true;
      openFirewall = true;
    };

    appimage = {
      enable = true;
      binfmt = true;
    };

    corectrl = { enable = true; };

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

    hyprlock = { enable = true; };

    nix-ld = { enable = true; };

    noisetorch.enable = true;

    steam = {
      enable = true;
      gamescopeSession.enable = true;
      # package = pkgs.steam;
    };

    wireshark.enable = true;

    virt-manager.enable = true;

    zsh.enable = true;
  };

  security = {
    polkit.enable = true;

    sudo.extraConfig = ''
      Defaults lecture="never"
    '';
  };

  services = {
    avahi = {
      enable = true;
      nssmdns4 = true;
      openFirewall = true;
    };

    # desktopManager = { cosmic.enable = true; };

    displayManager = {
      # cosmic-greeter.enable = true;

      sddm = {
        enable = true;
        wayland.enable = true;
        package = pkgs.kdePackages.sddm;
      };

      autoLogin = {
        enable = true;
        user = "blazej";
      };

      defaultSession = "hyprland";
    };

    flatpak.enable = true;

    fstrim.enable = true;

    gnome.gnome-keyring.enable = true;

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
      drivers = [ pkgs.brlaser ];
      # browsing = true;
      # defaultShared = true;
    };

    # thermald = {
    #   enable = true;
    #   ignoreCpuidCheck = true;
    # };

    upower.enable = true;

    xserver = {
      enable = true;
      xkb = {
        layout = "pl";
        variant = "";
      };
      desktopManager = {
        xterm.enable = false;
        xfce.enable = true;
      };
    };

    udev.extraRules = ''
      # ST-Link V3MINIE
      ATTRS{idVendor}=="0483", ATTRS{idProduct}=="3754", MODE="660", GROUP="plugdev", TAG+="uaccess"

      # Luxonis OAK cameras
      SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"

      # Oculus Quest
      SUBSYSTEM=="usb", ATTRS{idVendor}=="2833", ATTRS{idProduct}=="0186", MODE="0660", GROUP="plugdev", SYMLINK+="ocuquest%n"

      # Insta360 X4
      SUBSYSTEM=="usb", ATTRS{idVendor}=="2e1a", ATTRS{idProduct}=="0002", MODE="0666", SYMLINK+="insta%n"
      
      # eDRUMin 4
      SUBSYSTEM=="usb", ATTRS{idVendor}=="16c0", ATTRS{idProduct}=="0465", MODE="0666", SYMLINK+="edrumin%n"
    '';

    udev.packages = [ pkgs.openocd ];

    udisks2 = { enable = true; };

    teamviewer = { enable = true; };

  };

  specialisation = {
    rt-audio.configuration = { imports = [ ./specialisations/rt-audio.nix ]; };
  };

  # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
  system.stateVersion = "24.05";

  time.timeZone = "Europe/Warsaw";

  users = {
    groups = { plugdev = { }; };
    users = {
      blazej = {
        hashedPasswordFile = "/persist/passwords/blazej";
        isNormalUser = true;
        extraGroups = [
          "adbusers"
          "audio"
          "dialout"
          "lp"
          "networkmanager"
          "plugdev"
          "scanner"
          "wheel"
          "wireshark"
        ];
        shell = pkgs.zsh;
      };
    };
  };

  virtualisation = {
    libvirtd.enable = true;

    containers.enable = true;
    podman.enable = true;
  };

  xdg = {
    menus.enable = true;
    mime = {
      addedAssociations = {
        "x-scheme-handler/prusaslicer" = "PrusaSlicer.desktop";
      };
      defaultApplications = {
        "text/html" = "vivaldi-stable.desktop";
        "x-scheme-handler/http" = "vivaldi-stable.desktop";
        "x-scheme-handler/https" = "vivaldi-stable.desktop";
        "x-scheme-handler/about" = "vivaldi-stable.desktop";
        "x-scheme-handler/unknown" = "vivaldi-stable.desktop";
        "application/xhtml+xml" = "vivaldi-stable.desktop";
      };
    };
    portal = { xdgOpenUsePortal = true; };
  };
}
