{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    inputs.home-manager.nixosModules.home-manager
  ];

  boot = {
    blacklistedKernelModules = [ "ideapad_laptop" ];
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
  };

  console = {
    font = "Lat2-Terminus16";
    keyMap = "pl";
  };

  environment.systemPackages = with pkgs; [
    cryptsetup
    dolphin
    dunst
    element-desktop-wayland
    flameshot
    git
    grim
    htop
    kdePackages.qtwayland
    konsole
    light
    lshw
    pamixer
    pavucontrol
    playerctl
    pulseaudio
    pyprland
    python3
    rofi-wayland
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

      # Nvidia power management. Experimental, and can cause sleep/suspend to fail.
      # Enable this if you have graphical corruption issues or application crashes after waking
      # up from sleep. This fixes it by saving the entire VRAM memory to /tmp/ instead 
      # of just the bare essentials.
      powerManagement.enable = false;

      # Fine-grained power management. Turns off GPU when not in use.
      # Experimental and only works on modern Nvidia GPUs (Turing or newer).
      powerManagement.finegrained = false;

      # Use the NVidia open source kernel module (not to be confused with the
      # independent third-party "nouveau" open source driver).
      # Support is limited to the Turing and later architectures. Full list of 
      # supported GPUs is at: 
      # https://github.com/NVIDIA/open-gpu-kernel-modules#compatible-gpus 
      # Only available from driver 515.43.04+
      # Currently alpha-quality/buggy, so false is currently the recommended setting.
      open = false;

      # Enable the Nvidia settings menu,
      # accessible via `nvidia-settings`.
      nvidiaSettings = true;

      prime = {
        reverseSync.enable = true;
        offload = {
          enable = true;
          enableOffloadCmd = true;
        };

        nvidiaBusId = "PCI:1:0:0";
        amdgpuBusId = "PCI:5:0:0";
      };

      # Optionally, you may need to select the appropriate driver version for your specific GPU.
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
    gnupg.agent = {
      enable = true;
      enableSSHSupport = true;
    };

    hyprland = {
      enable = true;
      xwayland.enable = true;
    };
  };

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
      isNormalUser = true;
      openssh.authorizedKeys.keys = [
        # TODO: Add your SSH public key(s) here, if you plan on using SSH to connect
      ];
      extraGroups = [ "wheel" ];
    };
  };
}
