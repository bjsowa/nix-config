{ inputs, pkgs, lib, ... }:
let
  rtLinuxPackages = (pkgs.linuxPackagesFor
    (pkgs.linux_6_12.override {
      structuredExtraConfig = with lib.kernel; {
        EXPERT = yes;
        PREEMPT_RT = yes;
        RT_GROUP_SCHED = no;
      };
      ignoreConfigErrors = true;
    }));
  nvidiaOpenPackage = rtLinuxPackages.nvidiaPackages.beta.open.overrideAttrs
    (old: { makeFlags = old.makeFlags ++ [ "IGNORE_PREEMPT_RT_PRESENCE=1" ]; });
  nvidiaPackages = rtLinuxPackages.nvidiaPackages.beta // {
    open = nvidiaOpenPackage;
  };
in {
  imports = [ inputs.musnix.nixosModules.musnix ];

  boot = {
    kernelParams = lib.mkAfter [ "mitigations=off" ];
    kernelPackages = rtLinuxPackages;
  };

  hardware.nvidia.package = nvidiaPackages;

  musnix = {
    enable = true;

    rtcqs.enable = true;
    soundcardPciId = "06:00.6";
  };

  security.rtkit.enable = true;
}
