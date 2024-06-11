{ lib, pkgs, stdenv, makeWrapper, ... }:
stdenv.mkDerivation {
  name = "hypr-scripts";
  src = ./src;

  nativeBuildInputs = [ makeWrapper ];

  installPhase = ''
    mkdir -p $out/bin
    cp $src/* $out/bin
  '';

  postFixup = ''
    wrapProgram $out/bin/brightness.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          brightnessctl
          dunst # dunstify
          gawk # awk
        ]
      }"
    wrapProgram $out/bin/volume.sh \
      --set PATH "${
        with pkgs;
        lib.makeBinPath [
          coreutils # expr
          dunst
          libcanberra-gtk3
          pamixer
        ]
      }"
  '';
}
