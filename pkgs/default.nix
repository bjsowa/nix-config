# Custom packages, that can be defined similarly to ones from nixpkgs
# You can build them using 'nix build .#example'
pkgs: {
  waybar-scripts = pkgs.callPackage ./waybar-scripts { };
}
