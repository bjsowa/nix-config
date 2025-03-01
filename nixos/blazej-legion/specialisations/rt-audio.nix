{ inputs, pkgs, lib, ... }: {
  imports = [
    inputs.musnix.nixosModules.musnix
    # can't use nvidia with realtime kernel
    inputs.nixos-hardware.nixosModules.common-gpu-nvidia-disable
  ];

  boot.kernelParams = lib.mkAfter [ "mitigations=off" ];

  musnix = {
    enable = true;

    rtcqs.enable = true;
    soundcardPciId = "06:00.6";

    kernel = {
      realtime = true;
      packages = pkgs.linuxPackages_6_11_rt;
    };
  };
}
