{ config, lib, pkgs, ... }:
let
  inherit (lib) mkEnableOption mkIf mkOption mkPackageOption types;

  cfg = config.my.services.esphome;

  esphomeParams = "--address ${cfg.address} --port ${toString cfg.port}";

  ensureDir = lib.hm.dag.entryAfter [ "writeBoundary" ] ''
    mkdir -p ${lib.escapeShellArg cfg.stateDir}
  '';
in {
  options.my.services.esphome = {
    enable = mkEnableOption "ESPHome dashboard as a user service";

    package = mkPackageOption pkgs "esphome" { } // {
      default = pkgs.esphome;
      example = pkgs.esphome;
    };

    stateDir = mkOption {
      type = types.path;
      default = "${config.xdg.dataHome}/esphome";
      example = "${config.home.homeDirectory}/.local/share/esphome";
      description = "Directory holding ESPHome projects and build artifacts.";
    };

    address = mkOption {
      type = types.str;
      default = "localhost";
      description = "Address for the ESPHome dashboard to bind to.";
    };

    port = mkOption {
      type = types.int;
      default = 6052;
      description = "Port for the ESPHome dashboard to listen on.";
    };
  };

  config = mkIf cfg.enable {
    home.packages = [ cfg.package ];

    home.activation.ensureEspHomestateDir = ensureDir;

    systemd.user.services.esphome-dashboard = {
      Unit = {
        Description = "ESPHome Dashboard";
        After = [ "network-online.target" ];
        Wants = [ "network-online.target" ];
      };
      Service = {
        ExecStart =
          "${cfg.package}/bin/esphome dashboard ${esphomeParams} ${cfg.stateDir}";
        Restart = "on-failure";
      };
      Install.WantedBy = [ "default.target" ];
    };
  };
}
