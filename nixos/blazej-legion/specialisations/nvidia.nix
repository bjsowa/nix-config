{ inputs, lib, config, ... }: {
  imports = [ inputs.nixos-hardware.nixosModules.common-gpu-nvidia ];

  hardware.nvidia = {
    modesetting.enable = lib.mkDefault true;
    powerManagement.enable = lib.mkDefault false;
    powerManagement.finegrained = lib.mkDefault false;
    open = lib.mkDefault false;
    prime = {
      reverseSync.enable = lib.mkDefault true;
      offload = {
        enable = lib.mkDefault true;
        enableOffloadCmd = lib.mkDefault true;
      };
      amdgpuBusId = "PCI:5:0:0";
      nvidiaBusId = "PCI:1:0:0";
    };
    nvidiaSettings = true;
    package = config.boot.kernelPackages.nvidiaPackages.stable;
  };
}
