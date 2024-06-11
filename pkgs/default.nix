# Custom packages, that can be defined similarly to ones from nixpkgs
# You can build them using 'nix build .#example'
pkgs: {
  hypr-scripts = pkgs.callPackage ./hypr-scripts { };
  my-nixos-scripts = pkgs.callPackage ./my-nixos-scripts { };
  waybar-scripts = pkgs.callPackage ./waybar-scripts { };
}
