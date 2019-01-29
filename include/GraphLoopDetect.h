//
// Created by wang on 19-1-24.
//

#ifndef SPARSE3D_GRAPHLOOPDETECT_H
#define SPARSE3D_GRAPHLOOPDETECT_H
#include <stack>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <cassert>

class GraphLoopDetect
{
    int frame_num_;
public:
    std::vector<std::vector<int>> graph;

    // result variable, it will be filled after run DetectLoop()
    std::vector<std::vector<int>> edge_loop_num;
    std::vector<std::vector<int>> final_result;

    GraphLoopDetect(){}
    ~GraphLoopDetect(){}

    bool Init(std::string& traj_file, int frame_num);

    void  DetectLoop();

private:
    bool loop_eq(std::vector<int>& v1, std::vector<int>& v2);

};
#endif //SPARSE3D_GRAPHLOOPDETECT_H
