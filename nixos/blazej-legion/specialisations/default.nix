{ inputs, lib, config, ... }: {
  imports = [ inputs.nixos-hardware.nixosModules.common-gpu-nvidia-disable ];
}
