# 36-kore
Library for interfacing with ach channels on krang hardware

## Dependencies

- DART
 [Dart Homepage](https://dartsim.github.io)

- `libncurses5-dev`

      sudo apt install libncurses5-dev

- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic)
 Install the repo.

- [40-filter](https://github.gatech.edu/WholeBodyControlAttempt1/40-filter)
 Install the repo.

## Installation

Follow the instructions:

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/36-kore
    cd 36-kore
    git checkout newdart
    git pull origin newdart
    mkdir build
    cd build
    cmake ..
    sudo make install
