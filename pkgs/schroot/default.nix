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
    ++ [ ./no-setuid.patch ./no-pam-service.patch ];

  nativeBuildInputs = [ cmake ];

  buildInputs = [ boost gettext mandoc perl perlPackages.Po4a ];

  meta = with lib; {
    description = "Lightweight virtualisation tool";
    mainProgram = "schroot";
    homepage = "https://codeberg.org/shelter/reschroot";
    license = with licenses; [ gpl3 ];
  };
}
