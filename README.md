# 36-kore
Library for interfacing with ach channels on krang hardware

## Dependencies

- [DART](https://dartsim.github.io/install_dart_on_ubuntu.html) - Use 'apt install' instructions on the page.
- [37-somatic](https://github.gatech.edu/WholeBodyControlAttempt1/37-somatic) - Follow installation instructions on the git readme.
- [40-filter](https://github.gatech.edu/WholeBodyControlAttempt1/40-filter) - Clone, create build folder inside the locally cloned folder, then in that folder, cmake .. && make && make install
- libncurses5-dev

      sudo apt install libncurses5-dev

## Installation

Follow the instructions:

    git clone https://github.gatech.edu/WholeBodyControlAttempt1/36-kore
    cd 36-kore
    git checkout newdart
    git pull origin newdart
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
    sudo make install
