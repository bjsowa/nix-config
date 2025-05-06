{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    ./disko.nix
    inputs.impermanence.nixosModules.impermanence
    inputs.disko.nixosModules.default
  ];

  boot = {
    kernelPackages = lib.mkDefault pkgs.unstable.linuxPackages_6_13;

    loader = {
      efi.canTouchEfiVariables = true;
      systemd-boot.enable = true;
    };

    initrd.postDeviceCommands = lib.mkAfter ''
      mkdir /btrfs_tmp
      mount /dev/root_vg/root /btrfs_tmp

      delete_subvolume_recursively() {
        IFS=$'\n'
        for i in $(btrfs subvolume list -o "$1" | cut -f 9- -d ' '); do
          delete_subvolume_recursively "/btrfs_tmp/$i"
        done
        btrfs subvolume delete "$1"
      }

      if [[ -e /btrfs_tmp/old_root ]]; then
        delete_subvolume_recursively /btrfs_tmp/old_root
      fi

      if [[ -e /btrfs_tmp/root ]]; then
        mv /btrfs_tmp/root "/btrfs_tmp/old_root"
      fi

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
        "/var/lib/libvirt"
        "/var/lib/nixos"
        "/var/lib/systemd/coredump"
        "/srv/chroot"
      ];
      files = [ "/etc/machine-id" ];
    };
  };

  environment = {
    systemPackages = with pkgs; [
      autoconf
      automake
      clang-tools
      cmake
      cryptsetup
      debootstrap
      dmidecode
      exfatprogs
      file
      gcc
      gdb
      git
      gnumake
      htop
      keyutils
      lshw
      ncdu
      ninja
      nix-output-monitor
      nix-tree
      unstable.nixd
      nixfmt-classic
      nmap
      pkg-config
      python3
      screen
      sshfs
      unrar
      usbutils
      wget
      vim
    ];
  };

  fileSystems."/persist".neededForBoot = true;

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
    hostName = "ionix";
    firewall.enable = false;

    # We configure DHCP in systemd-networkd
    useDHCP = false;
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
    package = pkgs.unstable.nix;

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

    nix-ld = { enable = true; };

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
    locate.enable = true;

    openssh = {
      enable = true;
      settings = {
        PermitRootLogin = "no";
        PasswordAuthentication = false;
      };
    };
  };

  # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
  system.stateVersion = "24.11";

  systemd.network = {
    enable = true;
    networks = {
      "10-enp1s0" = {
        matchConfig.Name = "enp1s0";
        networkConfig.DHCP = "yes";
      };
    };
  };

  time.timeZone = "Europe/Warsaw";

  users = {
    users = {
      blazej = {
        initialPassword = "blazej";
        isNormalUser = true;
        extraGroups = [ "wheel" ];
      };
    };
  };

  virtualisation.libvirtd.enable = true;
}
