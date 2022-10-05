//
// Created by woons on 22. 9. 28.
//

#include "suvalgo.h"
#include <QDebug>

SUVAlgo::SUVAlgo(vector<QLineF> &mission_sets, vector<float> &EE_list) {
    this->mission_sets = mission_sets;
    this->EE_list = EE_list;
};

SUVAlgo::~SUVAlgo() {};

vector<vector<SUVAlgo::Wrapper>> SUVAlgo::make_matrix(vector<int> &group){

    vector<int> start;
    vector<int> end;
    for (int idx: group) {
        float sx = (this->mission_sets[idx].x1() * 100000);
        float ex = (this->mission_sets[idx].x2() * 100000);

        if (3676815.0 < sx && sx < 3676835.0) {
            start.push_back(VertiPort::POINT_A);
        } else if (3676340.0 < sx && sx < 3676350.0) {
            start.push_back(VertiPort::POINT_B);
        } else if (3676835.0 < sx && sx < 3676860.0) {
            start.push_back(VertiPort::POINT_C);
        } else if (3676395.0 < sx && sx < 3676420.0) {
            start.push_back(VertiPort::POINT_D);
        } else if (3676460.0 < sx && sx < 3676480.0) {
            start.push_back(VertiPort::POINT_E);
        } else {
            end.push_back(-1);
        }

        if (3676815.0 < ex && ex < 3676835.0) {
            end.push_back(VertiPort::POINT_A);
        } else if (3676340.0 < ex && ex < 3676350.0) {
            end.push_back(VertiPort::POINT_B);
        } else if (3676835.0 < ex && ex < 3676860.0) {
            end.push_back(VertiPort::POINT_C);
        } else if (3676395.0 < ex && ex < 3676420.0) {
            end.push_back(VertiPort::POINT_D);
        } else if (3676460.0 < ex && ex < 3676480.0) {
            end.push_back(VertiPort::POINT_E);
        } else {
            end.push_back(-1);
        }
    }

    vector <vector<SUVAlgo::Wrapper>> Matrix(this->verti_port_cnt,
                     vector<SUVAlgo::Wrapper>(this->verti_port_cnt, SUVAlgo::Wrapper()));

    for (int i = 0; i < group.size(); ++i) {

//        qDebug() << this->mission_sets[group[i]];
        float weight = this->mission_sets[group[i]].length() * this->EE_list[group[i]];

        if (Matrix[start[i]][end[i]].gradient == INF) {

            Matrix[start[i]][end[i]].id = group[i];
            Matrix[start[i]][end[i]].gradient = weight;
        } else {
            if (weight < Matrix[start[i]][end[i]].gradient) {
                Matrix[start[i]][end[i]].gradient = weight;
            }
        }
    }
//    for (int i = 0; i < this->verti_port_cnt; i++) {
//        for (int j = 0; j < this->verti_port_cnt; j++) {
//            cout << "id : " << Matrix[i][j].id << " / gradient : " << Matrix[i][j].gradient << endl;
//        }
//        cout << endl;
//    }
    return Matrix;
}

vector<vector<int>> SUVAlgo::solution(vector<int> &line_set){
    qDebug() << "[ UTM Solution Start ]";
    int n = this->mission_sets.size();
    vector<bool> visited(n, false);
    vector<bool> dp(pow(2, n), false);
    vector <vector<int>> res;

    this->grouping(0, line_set, visited, dp);
//    this->print_grouping();
    vector<int> group = this->choice_best_group();

//    qDebug() << group;

    while (!group.empty()) {
        vector<int> tmp;

        vector<vector<SUVAlgo::Wrapper>> wrapper_matrix = this->make_matrix(group);
        vector <vector<double>> matrix = vector<vector<double>>(wrapper_matrix.size(), vector<double>(wrapper_matrix[0].size(), 0));
        for (int i = 0; i < wrapper_matrix.size(); ++i) {
            for (int j = 0; j <wrapper_matrix[0].size(); ++j) {
                matrix[i][j] = (double)wrapper_matrix[i][j].gradient;
            }
        }
        vector<int> Assignment;
        HungarianAlgorithm hungarian = HungarianAlgorithm();
        vector<int> assign = hungarian.SolveV2(matrix, Assignment);


        for (int i = 0; i < assign.size(); i++) {
            if (matrix[i][assign[i]] == INF) {
                continue;
            }
//            qDebug() << i << "    " << matrix[i][assign[i]] << "  " << wrapper_matrix[i][assign[i]].id;
            tmp.push_back(wrapper_matrix[i][assign[i]].id);
            group.erase(remove(group.begin(), group.end(), wrapper_matrix[i][assign[i]].id), group.end());
        }
        res.push_back(tmp);
    }


    return res;
}

bool SUVAlgo::none_cross_check(vector<int> &line_set, QLineF cur_line) {
    for (int idx: line_set) {
        if(sementIntersects(this->mission_sets[idx], cur_line)) {
            return false;
        }
    }
    return true;
}

void SUVAlgo::grouping(int depth, vector<int> &line_set, vector<bool> &visited, vector<bool> &dp) {
    int key = this->make_key_set(line_set);
    if (dp[key]) {
        return;
    }

    dp[key] = true;

    if (line_set.size() >= this->max_cnt + 1) {
        this->groping_sets.clear();
        this->max_cnt = line_set.size();
    }
    if (line_set.size() >= this->max_cnt) {
        this->groping_sets.emplace_back(line_set);
    }

    for (int i = 0; i < this->mission_sets.size(); i++) {
        if (!visited[i] && this->none_cross_check(line_set, this->mission_sets[i])) {
            visited[i] = true;
            line_set.push_back(i);
            this->grouping(depth + 1, line_set, visited, dp);
            visited[i] = false;
            line_set.pop_back();
        }
    }
}

void SUVAlgo::print_grouping(){
    int i = 1;
    for (vector<int> set: this->groping_sets) {
        cout << "group " << i << " : ";
        for (int idx: set) {
            cout  << "/ line_idx : " << idx << "]";
        }
        cout << endl;
        i++;
    }
};

vector<int> SUVAlgo::choice_best_group() {
    int max = -1;
    int group_idx = 0;
    for (int i = 0; i < this->groping_sets.size(); i++) {
        float val = 0.0;
        for (int idx: this->groping_sets[i]) {
            val += this->mission_sets[idx].length() * this->EE_list[idx];
        }
        if (val > max) {
            max = val;
            group_idx = i;
        }
    }

    return this->groping_sets[group_idx];
}

int SUVAlgo::make_key_set(vector<int> &line_set) {
    int key = 0;
    for (int idx: line_set) {
        key |= (1 << idx);
    }
    return key;
}

