//
// Created by woons on 22. 9. 28.
//

#ifndef SUV_GCS_SUVAlgo_H
#define SUV_GCS_SUVAlgo_H

#include "mymath.h"
#include "hungarian.h"
#include <limits>

#include <vector>
#include <QLineF>
#include <iostream>

using namespace std;

#define INF numeric_limits<float>::max()

enum VertiPort {
    POINT_A,
    POINT_B,
    POINT_C,
    POINT_D,
    POINT_E
};

class SUVAlgo
{
public:
    class Wrapper{
    public:
        Wrapper(int id, float gradient){
            this->id = id;
            this->gradient = gradient;
        }

        Wrapper(){
            this->id = -1;
            this->gradient = INF;
        };

        int id;
        float gradient;
    };

    SUVAlgo();
    SUVAlgo(vector <QLineF> &mission_sets, vector<float> &EE_list);
    ~SUVAlgo();

    int verti_port_cnt = 5;
    vector<vector<int>> groping_sets;
    int max_cnt = -1;

    vector<QLineF> mission_sets;
    vector<float> EE_list;


    void print_grouping();
    vector <vector<int>> solution(vector<int> &line_set);
    void test();

private:

    int make_key_set(vector<int> &line_set);

    vector<int> choice_best_group();
    bool none_cross_check(vector<int> &line_set, QLineF cur_line);
    vector<vector<SUVAlgo::Wrapper>> make_matrix(vector<int> &group);
    vector<vector<SUVAlgo::Wrapper>> make_test_matrix(vector<int> &group);
    void grouping(int depth, vector<int> &line_set, vector<bool> &visited, vector<bool> &dp);

};



#endif //SUV_GCS_SUVAlgo_H
