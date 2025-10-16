{ inputs, outputs, config, lib, pkgs, ... }: {

  home = {
    username = "blazej";
    homeDirectory = "/home/${config.home.username}";

    # https://nixos.wiki/wiki/FAQ/When_do_I_update_stateVersion
    stateVersion = "24.11";
  };

  programs = {
    bash.enable = true;
    direnv = {
      enable = true;
      enableBashIntegration = true;
      nix-direnv.enable = true;
    };
    git = {
      enable = true;
      lfs.enable = true;
      userName = "Błażej Sowa";
      userEmail = "bsowa123@gmail.com";
      extraConfig = {
        url."ssh://git@github.com/" = {
          pushInsteadOf = "https://github.com/";
        };
      };
    };
    home-manager.enable = true;
    pyenv = {
      enable = true;
      enableBashIntegration = true;
    };
    zsh = {
      enable = true;
      enableCompletion = true;
      autosuggestion.enable = true;
      syntaxHighlighting.enable = true;
      history = {
        size = 10000;
        path = "${config.xdg.dataHome}/zsh/history";
      };
      oh-my-zsh = {
        enable = true;
        custom =
          "${config.home.homeDirectory}/.local/share/datafiles/oh-my-zsh";
        theme = "agnoster-custom";
        plugins = [
          "aliases"
          "common-aliases"
          "colored-man-pages"
          "copybuffer"
          "dircycle"
          "git"
          "last-working-dir"
          "magic-enter"
          "nmap"
          "safe-paste"
          "scd"
          "sudo"
          "timer"
          "transfer"
          "zbell"
        ];
      };
      envExtra = ''
        export DEFAULT_USER=${config.home.username}
      '';
    };
  };

  xdg = {
    dataFile = {
      "datafiles".source = config.lib.file.mkOutOfStoreSymlink
        "${config.home.homeDirectory}/nix-config/datafiles";
    };
  };
}
