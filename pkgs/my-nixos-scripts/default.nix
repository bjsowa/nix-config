{ lib, pkgs, stdenv, makeWrapper, ... }:
stdenv.mkDerivation {
  name = "my-nixos-scripts";
  src = ./src;

  nativeBuildInputs = [ makeWrapper ];

  propagatedBuildInputs = [
    (pkgs.python3.withPackages
      (pythonPackages: with pythonPackages; [ requests ]))
  ];

  installPhase = ''
    mkdir -p $out
    cp -r $src/* $out
  '';

  postFixup = ''
    wrapProgram $out/dunst/playNotificationSound.sh \
      --set PATH "${with pkgs; lib.makeBinPath [ libcanberra-gtk3 ]}"
    wrapProgram $out/hypr/brightness.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          brightnessctl
          dunst # dunstify
          gawk # awk
        ]
      }"
    wrapProgram $out/hypr/volume.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          coreutils # expr
          dunst
          libcanberra-gtk3
          pamixer
        ]
      }"
    wrapProgram $out/waybar/powermenu.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          procps # uptime

          alsa-utils # amixer
          coreutils # cat, cut
          gnused # sed
          hyprland # hyprctl
          playerctl
          rofi-wayland
          swaylock-effects # swaylock
          systemd # systemctl
        ]
      }"
    wrapProgram $out/waybar/rofi-bluetooth.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          bc
          bluez # bluetoothctl
          coreutils # sleep
          gnugrep # grep
          procps # pgrep
          rofi-wayland
          util-linux # rfkill, kill
        ]
      }"
  '';
}
