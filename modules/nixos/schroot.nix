{ config, lib, pkgs, ... }:

let cfg = config.programs.schroot;
in {
  options = {
    programs.schroot = {
      enable = lib.mkEnableOption "Schroot";
      package = lib.mkPackageOption pkgs "schroot" { };
    };
  };

  config = lib.mkIf cfg.enable {
    environment = {
      systemPackages = [ cfg.package ];
      etc."schroot/chroot.d/.keep".text = "";
    };

    security.wrappers.schroot = {
      source = "${cfg.package}/bin/schroot";
      owner = "root";
      group = "root";
      setuid = true;
    };

    systemd.tmpfiles.rules = [
      "d /var/lib/schroot/session - root root - -"
      "d /var/lib/schroot/unpack - root root - -"
      "d /var/lib/schroot/union - root root - -"
      "d /var/lib/schroot/union/overlay - root root - -"
      "d /var/lib/schroot/union/underlay - root root - -"
    ];
  };
}
