//
// Created by woons on 22. 9. 28.
//

#ifndef SUV_GCS_SWALGO_H
#define SUV_GCS_SWALGO_H

#include "mymath.h"
#include <vector>
#include <QLineF>
#include <iostream>
using namespace std;

class SWAlgo
{
public:
    SWAlgo();
    ~SWAlgo();

    vector<vector<int>> groping_set;
    int max_cnt = -1;

    void print_grouping(vector<int> &sysid_list);
    void grouping(int depth, vector<QLineF> &mission_set, vector<int> &line_set,
                  vector<bool> &visited, vector<bool> &dp);

    vector<int> choice_best_group(vector <QLineF> &mission_set, vector<float> &EE_list);

private:

    int make_key_set(vector<int> &line_set);

    bool none_cross_check(vector <QLineF> &mission_set, vector<int> &line_set, QLineF cur_line);
};



#endif //SUV_GCS_SWALGO_H
