{
  lib,
  omniorb,
  pkg-config,
  cmake,
  python3Packages,
  doxygen,
}:

python3Packages.buildPythonPackage rec {
  pname = "hpp-task-sequencing";
  version = "unstable-2024-05-07";
  pyproject = false;

  src = lib.fileset.toSource {
    root = ./.;
    fileset = lib.fileset.unions [
      ./CMakeLists.txt
      ./corba
      ./include
      ./script.py
      ./src
    ];
  };

  outputs = [
    "doc"
    "out"
  ];

  enableParallelBuilding = false;

  nativeBuildInputs = [
    doxygen
    cmake
    omniorb
    pkg-config
  ];
  propagatedBuildInputs = [ python3Packages.hpp-corbaserver ];

  meta = {
    description = "Compute sequence of motions to achieve a set of tasks";
    homepage = "https://github.com/florent-lamiraux/hpp-task-sequencing";
    license = lib.licenses.bsd2;
    maintainers = with lib.maintainers; [ nim65s ];
  };
}
