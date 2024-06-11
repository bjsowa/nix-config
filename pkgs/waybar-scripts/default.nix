{ lib, pkgs, stdenv, makeWrapper, ... }:
stdenv.mkDerivation {
  name = "waybar-scripts";
  src = ./src;

  nativeBuildInputs = [ makeWrapper ];

  propagatedBuildInputs = [
    (pkgs.python3.withPackages
      (pythonPackages: with pythonPackages; [ requests ]))
  ];

  installPhase = ''
    mkdir -p $out/bin
    cp $src/* $out/bin
  '';

  postFixup = ''
    wrapProgram $out/bin/powermenu.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          procps # uptime

          alsa-utils # amixer
          coreutils # cat, cut
          gnused # sed
          hyprland # hyprctl
          playerctl
          rofi
          swaylock-effects # swaylock
          systemd # systemctl
        ]
      }"
    wrapProgram $out/bin/rofi-bluetooth.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          bc
          bluez # bluetoothctl
          coreutils # sleep
          gnugrep # grep
          procps # pgrep
          rofi
          util-linux # rfkill, kill
        ]
      }"
  '';
}
