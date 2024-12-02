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
    powerManagement.enable = lib.mkDefault false;
    powerManagement.finegrained = lib.mkDefault false;
    open = lib.mkDefault true;
    nvidiaSettings = lib.mkDefault true;
    package = lib.mkDefault config.boot.kernelPackages.nvidiaPackages.beta;
  };

  services.xserver.videoDrivers = lib.mkAfter [ "nvidia" ];
}
