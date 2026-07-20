# Custom packages, that can be defined similarly to ones from nixpkgs
# You can build them using 'nix build .#example'
pkgs: {
  ccstudio = pkgs.callPackage ./ccstudio { };
  my-nixos-scripts = pkgs.callPackage ./my-nixos-scripts { };
}
