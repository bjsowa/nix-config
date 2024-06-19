{ lib, stdenv, fetchurl, pkgs }:

let
  scripts-bin-path = with pkgs;
    lib.makeBinPath [ coreutils getent gnugrep gnused gnutar util-linux ];

in stdenv.mkDerivation rec {
  pname = "schroot";
  version = "1.6.13";

  src = fetchurl {
    url =
      "https://codeberg.org/shelter/reschroot/archive/release/reschroot-${version}.tar.gz";
    hash = "sha256-wF1qG7AhDUAeZSLu4sRl4LQ8bJj3EB1nH56e+Is6zPU=";
  };

  patches = map fetchurl (import ./debian-patches.nix) ++ [
    ./no-setuid.patch
    ./no-pam-service.patch
    ./no-default-config.patch
    ./fix-absolute-paths.patch
  ];

  nativeBuildInputs = with pkgs; [ cmake findutils makeWrapper ];

  buildInputs = with pkgs; [ boost gettext mandoc perl perlPackages.Po4a ];

  cmakeFlags = [
    "-DCMAKE_INSTALL_LOCALSTATEDIR=/var"
    "-DSCHROOT_SYSCONF_DIR=/etc/schroot"
    "-DSCHROOT_CONF_SETUP_D=${placeholder "out"}/etc/schroot/setup.d"
  ];

  postPatch = ''
    substituteInPlace bin/schroot-mount/schroot-mount-main.cc \
      --replace-fail "/bin/mount" "${pkgs.util-linux}/bin/mount"
  '';

  postFixup = ''
    mkdir $out/etc/schroot/setup.d.wrapped
    cd $out/etc/schroot/setup.d
    find * -type f | while read -r file; do
      mv "$file" $out/etc/schroot/setup.d.wrapped
      makeWrapper "$out/etc/schroot/setup.d.wrapped/$file" "$file" --set PATH ${scripts-bin-path}
    done
  '';

  meta = with lib; {
    description = "Lightweight virtualisation tool";
    mainProgram = "schroot";
    homepage = "https://codeberg.org/shelter/reschroot";
    license = with licenses; [ gpl3 ];
  };
}
