//
// Created by wang on 19-1-14.
//

#include "sift.h"
//#include <iostream>
#include <boost/filesystem.hpp>

using namespace std;

int main(int argc, char** argv) {
    string input;
    string output;
    input = argv[1];
    output = argv[2];
    sift(input,output);
}
