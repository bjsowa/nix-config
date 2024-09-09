{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ inputs.musnix.nixosModules.musnix ];

  boot = {
    kernelPackages = pkgs.master.linuxPackages_6_6;
    kernelParams = lib.mkAfter [ "mitigations=off" ];
  };

  musnix = {
    enable = true;

    rtcqs.enable = true;
    soundcardPciId = "06:00.6";
  };

  hardware.bluetooth.enable = lib.mkForce false;
}
