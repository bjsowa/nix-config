{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [
    ./hardware-configuration.nix
    ./disko.nix
    inputs.home-manager.nixosModules.home-manager
    inputs.impermanence.nixosModules.impermanence
    inputs.disko.nixosModules.default
  ];

  boot = {
    kernelPackages = lib.mkDefault pkgs.linuxPackages_6_14;

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
      users.blazej = {
        directories = [
          "Documents"

          "nix-config"

          ".cache"
          ".gnupg"
          ".ssh"

          ".local/share/zsh"
        ];
        files = [ ".bash_history" ".scdhistory" ];
      };
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
      nixd
      nixfmt-classic
      nmap
      pkg-config
      python3
      screen
      sshfs
      unrar
      usbutils
      wget
      wireguard-tools
      vim
    ];
  };

  fileSystems."/persist".neededForBoot = true;

  home-manager = {
    extraSpecialArgs = { inherit inputs outputs; };
    users = { blazej = import ../../home-manager/ionix/blazej.nix; };
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
    hostName = "ionix";

    firewall = {
      enable = true;
      allowedUDPPorts = [ 51820 ];
    };

    nat = {
      enable = true;
      externalInterface = "enp1s0";
      internalInterfaces = [ "wg0" ];
    };

    wireguard = {
      enable = true;
      interfaces = {
        wg0 = {
          ips = [ "10.100.0.1/24" ];
          listenPort = 51820;
          privateKeyFile = "/persist/secrets/wg-private";
          peers = [
            {
              name = "blazej-legion";
              publicKey = "bl/3slnkZnl3A1W+dlfTp624ykKsDukiFY00rca1t28=";
              allowedIPs = [ "10.100.0.2/32" ];
            }
            {
              name = "blazej-home-mikrotik";
              publicKey = "hm/ngzQeP365+qP5ehWNgk28XDJtewWGQqMMrCkFK1k=";
              allowedIPs = [ "10.100.0.3/32" ];
            }
          ];
        };
      };
    };

    nftables.enable = true;
    useNetworkd = true;
    useDHCP = true;
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
        PermitRootLogin = "yes";
        PasswordAuthentication = false;
      };
      hostKeys = [
        {
          bits = 4096;
          path = "/persist/secrets/ssh_host_rsa_key";
          type = "rsa";
        }
        {
          path = "/persist/secrets/ssh_host_ed25519_key";
          type = "ed25519";
        }
      ];
    };
  };

  # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
  system.stateVersion = "24.11";

  time.timeZone = "Europe/Warsaw";

  users = {
    users = let
      blazej-legion-public-key =
        "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIF5ka8MWsrKaPsywWPZNiEVHzHKNf0x2Vzk6uIkNSMbr blazej@blazej-legion";
    in {
      blazej = {
        hashedPasswordFile = "/persist/secrets/blazej-hashed-password";
        isNormalUser = true;
        extraGroups = [ "wheel" ];
        shell = pkgs.zsh;
        openssh.authorizedKeys.keys = [ blazej-legion-public-key ];
      };
      root = { openssh.authorizedKeys.keys = [ blazej-legion-public-key ]; };
    };
  };

  virtualisation.libvirtd.enable = true;
}
