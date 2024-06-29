{ inputs, outputs, lib, config, pkgs, ... }: {
  imports = [ inputs.musnix.nixosModules.musnix ];

  musnix = {
    enable = true;
    kernel = {
      realtime = true;
      packages = pkgs.linuxPackages_6_8_rt;
    };
  };
}
