# Custom packages, that can be defined similarly to ones from nixpkgs
# You can build them using 'nix build .#example'
pkgs: {
  my-nixos-scripts = pkgs.callPackage ./my-nixos-scripts { };
  schroot = pkgs.callPackage ./schroot { };
}
