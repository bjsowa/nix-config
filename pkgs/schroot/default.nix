{ lib, stdenv, fetchurl, boost, cmake, gettext, mandoc, perl, perlPackages }:

stdenv.mkDerivation rec {
  pname = "schroot";
  version = "1.6.13";

  src = fetchurl {
    url =
      "https://codeberg.org/shelter/reschroot/archive/release/reschroot-${version}.tar.gz";
    hash = "sha256-wF1qG7AhDUAeZSLu4sRl4LQ8bJj3EB1nH56e+Is6zPU=";
  };

  patches = map fetchurl (import ./debian-patches.nix)
    ++ [ ./no-setuid.patch ./no-pam-service.patch ./no-default-config.patch ];

  nativeBuildInputs = [ cmake ];

  buildInputs = [ boost gettext mandoc perl perlPackages.Po4a ];

  cmakeFlags = [
    "-DSCHROOT_SYSCONF_DIR=/etc/schroot"
    "-DSCHROOT_CONF_SETUP_D=${placeholder "out"}/etc/schroot/setup.d"
  ];

  meta = with lib; {
    description = "Lightweight virtualisation tool";
    mainProgram = "schroot";
    homepage = "https://codeberg.org/shelter/reschroot";
    license = with licenses; [ gpl3 ];
  };
}
