//
// Created by woons on 22. 9. 28.
//

#include "swalgo.h"
#include <QDebug>
SWAlgo::SWAlgo() {};
SWAlgo::~SWAlgo() {};


bool SWAlgo::none_cross_check(vector<QLineF> &mission_set, vector<int> &line_set, QLineF cur_line) {
    for (int idx: line_set) {
        if(sementIntersects(mission_set[idx], cur_line)) {
            return false;
        }
    }
    return true;
}

int SWAlgo::make_key_set(vector<int> &line_set) {
    int key = 0;
    for (int idx: line_set) {
        key |= (1 << idx);
    }
    return key;
}

void SWAlgo::grouping(int depth, vector<QLineF> &mission_set, vector<int> &line_set,
                        vector<bool> &visited, vector<bool> &dp) {
    int key = this->make_key_set(line_set);
    if (dp[key]) {
        return;
    }

    dp[key] = true;

    for (int l: line_set) {
        cout << l;
    }
    cout << endl;
    if (line_set.size() >= this->max_cnt + 1) {
        this->groping_set.clear();
        this->max_cnt = line_set.size();
    }
    if (line_set.size() >= this->max_cnt) {
        this->groping_set.emplace_back(line_set);
    }

    for (int i = 0; i < mission_set.size(); i++) {
        if (!visited[i] && this->none_cross_check(mission_set, line_set, mission_set[i])) {
            visited[i] = true;
            line_set.push_back(i);
            this->grouping(depth + 1, mission_set, line_set, visited, dp);
            visited[i] = false;
            line_set.pop_back();
        }
    }
}

void SWAlgo::print_grouping(vector<int> &sysid_list){
    int i = 1;
    for (vector<int> set: this->groping_set) {
        cout << "group " << i << " : ";
        for (int idx: set) {
            cout << "[sys_id : " << sysid_list[idx] << "/ line : " << idx << "]";
        }
        cout << endl;
        i++;
    }
};

vector<int> SWAlgo::choice_best_group(vector<QLineF> &mission_set, vector<float> &EE_list) {
    int max = -1;
    int group_idx = -1;
    for (int i = 0; i < this->groping_set.size(); i++) {
        //TODO
        float val = 0.0;
        for (int idx: this->groping_set[i]) {
            int line_idx = this->groping_set[i][idx];
//            qDebug() << mission_set[line_idx].length();
            val += (float)mission_set[line_idx].length() * EE_list[idx];
        }
        if (val > max) {
            max = val;
            group_idx = i;
        }
    }
    return this->groping_set[group_idx];
}

