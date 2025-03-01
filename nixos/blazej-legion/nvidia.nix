{ lib, config, ... }: {

  boot = {
    extraModprobeConfig = lib.mkAfter ''
      blacklist nouveau
      options nouveau modeset=0
    '';

    kernelParams = lib.mkAfter [ "nvidia_drm.fbdev=1" ];

    initrd.kernelModules = lib.mkAfter [ "nvidia" ];
  };

  hardware.nvidia = {
    modesetting.enable = lib.mkDefault true;
    powerManagement.enable = lib.mkDefault true;
    powerManagement.finegrained = lib.mkDefault true;
    open = lib.mkDefault true;
    nvidiaSettings = lib.mkDefault true;
    package = lib.mkDefault config.boot.kernelPackages.nvidiaPackages.beta;

    prime = {
      offload = {
        enable = true;
        enableOffloadCmd = true;
      };
      amdgpuBusId = "PCI:5:0:0";
      nvidiaBusId = "PCI:1:0:0";
    };
  };

  services.xserver.videoDrivers = lib.mkAfter [ "nvidia" ];
}
