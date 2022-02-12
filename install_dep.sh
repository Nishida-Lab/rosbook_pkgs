vcs import src < src/rosbook_pkgs/.kinetic_depends.rosinstall
vsc pull src
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
  ros-kinetic-hardware-interface \
  ros-kinetic-controller-interface \
  ros-kinetic-convex-decomposition \
  ros-kinetic-ivcon \
  ros-kinetic-control-toolbox