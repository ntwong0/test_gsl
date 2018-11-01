# Compilation note:

To compile, do the following:
<!-- gcc <filename> -lgsl -lgslcblas -->
g++ -std=c++11 src/file.cpp -lgsl -lgslcblas -Iinclude

Header files of the gsl library are located at /usr/include/gsl
