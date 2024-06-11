{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    inputs.impermanence.nixosModules.impermanence
    inputs.home-manager.nixosModules.home-manager
  ];

  boot = {
    blacklistedKernelModules = [ "ideapad_laptop" ];
    kernelPackages = pkgs.linuxPackages_6_9;
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
    font = "Lat2-Terminus16";
    keyMap = "pl";
  };

  environment.persistence."/persist/system" = {
    hideMounts = true;
    directories = [
      "/var/cache"
      "/var/log"
      "/var/lib/bluetooth"
      "/var/lib/nixos"
      "/var/lib/systemd/coredump"
      "/etc/NetworkManager/system-connections"
    ];
    files = [ "/etc/machine-id" ];
  };

  environment.systemPackages = with pkgs; [
    caprine-bin
    cryptsetup
    dmidecode
    dolphin
    dunst
    element-desktop
    flameshot
    git
    grim
    htop
    kdePackages.qtwayland
    konsole
    light
    lm_sensors
    lshw
    pamixer
    pavucontrol
    playerctl
    pulseaudio
    pyprland
    python3
    rofi-wayland
    slack
    swaybg
    swaylock
    waybar
    wdisplays
    wget
    vim
    vivaldi
    vivaldi-ffmpeg-codecs
    vscode
  ];

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

    nvidia = {
      modesetting.enable = true;
      nvidiaSettings = true;
      open = false;
      powerManagement.enable = true;
      powerManagement.finegrained = true;

      prime = {
        reverseSync.enable = true;
        offload = {
          enable = true;
          enableOffloadCmd = true;
        };

        nvidiaBusId = "PCI:1:0:0";
        amdgpuBusId = "PCI:5:0:0";
      };

      package = config.boot.kernelPackages.nvidiaPackages.stable;
    };
  };

  home-manager = {
    extraSpecialArgs = { inherit inputs outputs; };
    users = { blazej = import ../../home-manager/blazej/home.nix; };
  };

  i18n.defaultLocale = "en_US.UTF-8";

  networking = {
    hostName = "blazej-legion";
    networkmanager.enable = true;
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
    };
    # Opinionated: disable channels
    channel.enable = false;

    # Opinionated: make flake registry and nix path match flake inputs
    registry = lib.mapAttrs (_: flake: { inherit flake; }) flakeInputs;
    nixPath = lib.mapAttrsToList (n: _: "${n}=flake:${n}") flakeInputs;
  };

  programs = {
    fuse.userAllowOther = true;

    gnupg.agent = {
      enable = true;
      enableSSHSupport = true;
    };

    hyprland = {
      enable = true;
      xwayland.enable = true;
    };

    nix-ld = { enable = true; };
  };

  security.pam.services.swaylock = { };
  security.polkit.enable = true;

  services = {

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

    printing = {
      enable = true;
      drivers = [ ];
      # browsing = true;
      # defaultShared = true;
    };

    xserver = {
      enable = true;
      xkb = {
        layout = "pl";
        variant = "";
      };
      videoDrivers = [ "nvidia" ];
    };

    udisks2 = { enable = true; };

  };

  sound.enable = true;

  # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
  system.stateVersion = "24.05";

  time.timeZone = "Europe/Warsaw";

  users.users = {
    blazej = {
      hashedPasswordFile = "/persist/passwords/blazej";
      isNormalUser = true;
      extraGroups = [ "networkmanager" "wheel" ];
    };
  };
}
