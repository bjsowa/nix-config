{ pkgs, ... }: {

  programs.schroot = {
    enable = true;
    settings = {
      "focal" = {
        type = "directory";
        description = "Ubuntu 20.04 Focal Fossa";
        directory = "/srv/chroot/focal";
        users = "blazej";
        root-users = "blazej";
        personality = "linux";
        preserve-environment = false;
        profile = "my-profile";
        shell = "/bin/zsh";
      };
      "jammy" = {
        type = "directory";
        description = "Ubuntu 22.04 Jammy Jellyfish";
        directory = "/srv/chroot/jammy";
        users = "blazej";
        root-users = "blazej";
        personality = "linux";
        preserve-environment = false;
        profile = "my-profile";
        shell = "/bin/zsh";
      };
      "noble" = {
        type = "directory";
        description = "Ubuntu 24.04 Noble";
        directory = "/srv/chroot/noble";
        users = "blazej";
        root-users = "blazej";
        personality = "linux";
        preserve-environment = false;
        profile = "my-profile";
        shell = "/bin/zsh";
      };
    };

    profiles = {
      "my-profile" = {
        copyfiles = [ "/etc/resolv.conf" ];
        fstab = pkgs.writeText "fstab" ''
          /proc           /proc           none    rw,bind         0       0
          /sys            /sys            none    rw,bind         0       0
          /dev            /dev            none    rw,bind         0       0
          /dev/pts        /dev/pts        none    rw,bind         0       0
          /home           /home           none    rw,rbind        0       0
          /tmp            /tmp            none    rw,bind         0       0
          /dev/shm        /dev/shm        none    rw,bind         0       0
          /nix            /nix            none    ro,bind         0       0
          /persist        /persist        none    ro,bind         0       0
          /run/binfmt     /run/binfmt none rw,bind    0       0
          /run/current-system /run/current-system none rw,bind    0       0
          /run/wrappers   /run/wrappers   none    rw,bind         0       0
          /run/user/1000  /run/user/1000  none    rw,bind,uid=1000,gid=1000         0       0
          /proc/sys/fs/binfmt_misc /proc/sys/fs/binfmt_misc none    rw,bind         0       0
        '';
        nssdatabases = [ "services" "protocols" "hosts" ];
      };
    };
  };
}
