{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ inputs.musnix.nixosModules.musnix ];

  boot = { kernelPackages = pkgs.master.linuxPackages_6_6; };

  musnix = {
    enable = true;
    # kernel = {
    # realtime = true;
    # packages = pkgs.linuxPackages_6_9_rt;
    # };
  };

  hardware.bluetooth.enable = lib.mkForce false;
}
