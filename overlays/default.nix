{ inputs, ... }: {
  additions = final: _prev: import ../pkgs final.pkgs;

  modifications = final: prev: {
    caprine-bin = prev.appimageTools.wrapType2 rec {
      pname = "caprine";
      version = "2.60.1";
      src = prev.fetchurl {
        url =
          "https://github.com/sindresorhus/caprine/releases/download/v${version}/Caprine-${version}.AppImage";
        name = "Caprine-${version}.AppImage";
        sha256 = "sha256-qeJZ3DoeI46/JUJq0jA6HiefzI8AlxdbTuDhTiLmZ8w=";
      };

      extracted =
        prev.appimageTools.extractType2 { inherit pname version src; };

      passthru = { inherit pname version src; };

      profile = ''
        export LC_ALL=C.UTF-8
      '';

      extraInstallCommands = ''
        mkdir -p $out/share
        "${prev.xorg.lndir}/bin/lndir" -silent "${extracted}/usr/share" "$out/share"
        ln -s ${extracted}/caprine.png $out/share/icons/caprine.png
        mkdir $out/share/applications
        cp ${extracted}/caprine.desktop $out/share/applications/
        substituteInPlace $out/share/applications/caprine.desktop \
            --replace AppRun caprine
      '';
    };

    flameshot = prev.flameshot.overrideAttrs (old: {
      src = prev.fetchFromGitHub {
        owner = "flameshot-org";
        repo = "flameshot";
        rev = "a1dda59108d420a26d603a40d1c7d25e4114d748";
        sha256 = "sha256-uISujW2Bqv07MpxCj9sbms//cJQJNRI+uJmkUphl1ds=";
      };
      cmakeFlags = old.cmakeFlags
        ++ [ (prev.lib.cmakeBool "USE_WAYLAND_GRIM" true) ];
    });
  };

  unstable-packages = final: _prev: {
    unstable = import inputs.nixpkgs-unstable {
      system = final.system;
      config.allowUnfree = true;
    };
    master = import inputs.nixpkgs-master {
      system = final.system;
      config.allowUnfree = true;
    };
    old = import inputs.nixpkgs-old {
      system = final.system;
      config.allowUnfree = true;
    };
    hyprland-git = inputs.hyprland.packages.${final.system};
  };
}
