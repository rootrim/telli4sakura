{
  description = "Code of Telli IV: Sakura";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    idf.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = {
    self,
    nixpkgs,
    idf,
  }: {
    devShells.x86_64-linux.default = let
      pkgs = nixpkgs.legacyPackages.x86_64-linux;
      system = "x86_64-linux";
      theidf = idf.packages.${system}.esp-idf-full;
    in
      pkgs.mkShell {
        buildInputs = with pkgs; [
          theidf
          cmake
          ninja
          python3
        ];
      };
  };
}
