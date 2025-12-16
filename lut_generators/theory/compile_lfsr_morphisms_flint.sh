g++ -std=c++17 -O3 -march=native -pthread \
    -I/path/to/nlohmann -I/usr/include -L/usr/lib \
    lfsr_morphism_flint.cpp -o lfsr_morphism_flint \
    -lflint -lgmp -lmpfr
