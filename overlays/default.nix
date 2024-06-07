{ inputs, ... }: {
  additions = final: _prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    flameshot = prev.flameshot.overrideAttrs (old: {
      src = prev.fetchFromGitHub {
        owner = "flameshot-org";
        repo = "flameshot";
        rev = "a1dda59108d420a26d603a40d1c7d25e4114d748";
        sha256 = "sha256-uISujW2Bqv07MpxCj9sbms//cJQJNRI+uJmkUphl1ds=";
      };
    });
  };

  unstable-packages = final: _prev: {
    unstable = import inputs.nixpkgs-unstable {
      system = final.system;
      config.allowUnfree = true;
    };
  };
}
