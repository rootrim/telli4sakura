{
  description = "Code of Telli IV: Sakura";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    idf.url = "github:mirrexagon/nixpkgs-esp-dev";
    naersk.url = "github:nix-community/naersk";
    naersk.inputs.nixpkgs.follows = "nixpkgs";
    naersk.inputs.fenix.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs = {
    nixpkgs,
    idf,
    naersk,
    ...
  }: let
    system = "x86_64-linux";
    pkgs = nixpkgs.legacyPackages.${system};
    theidf = idf.packages.${system}.esp-idf-full;
    naersk' = pkgs.callPackage naersk {};
  in {
    devShells.${system}.default = pkgs.mkShell {
      buildInputs = with pkgs; [
        theidf
        cmake
        ninja
        python3
      ];
    };

    packages.${system}.gru = naersk'.buildPackage {
      pname = "gru";
      version = "unstable";
      src = ./gru;

      nativeBuildInputs = with pkgs; [pkg-config];
      buildInputs = with pkgs; [udev];
    };
  };
}
