{ config, lib, pkgs, modulesPath, ... }:

{
  imports = [ (modulesPath + "/profiles/qemu-guest.nix") ];

  boot.initrd.availableKernelModules = [ "xhci_pci" "virtio_scsi" "sr_mod" ];
  boot.initrd.kernelModules = [ ];
  boot.kernelModules = [ ];
  boot.extraModulePackages = [ ];

  fileSystems."/" = {
    device = "/dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e";
    fsType = "btrfs";
    options = [ "subvol=root" ];
  };

  # fileSystems."/old_roots" = {
  #   device = "/dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e";
  #   fsType = "btrfs";
  #   options = [ "subvol=old_roots" ];
  # };

  # fileSystems."/boot" = {
  #   device = "/dev/disk/by-uuid/8741-BFA8";
  #   fsType = "vfat";
  #   options = [ "fmask=0022" "dmask=0022" ];
  # };

  # fileSystems."/nix" = {
  #   device = "/dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e";
  #   fsType = "btrfs";
  #   options = [ "subvol=nix" ];
  # };

  fileSystems."/persist" = {
    device = "/dev/disk/by-uuid/0184e3ee-792a-406f-98ea-ec99a16c6c5e";
    fsType = "btrfs";
    options = [ "subvol=persist" ];
  };

  # swapDevices =
  #   [{ device = "/dev/disk/by-uuid/1563cd08-9a71-4a87-8d93-5fdb28b727c0"; }];

  nixpkgs.hostPlatform = lib.mkDefault "aarch64-linux";
}
