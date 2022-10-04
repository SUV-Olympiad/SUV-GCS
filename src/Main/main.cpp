//#include <QtGui/QApplication>
#include "mainwidget.h"
#include <QApplication>
#include <stdio.h> 
#include "manager.h"
#include <GL/glut.h>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp> //임시


#include "suvalgo.h"
#include "hungarian.h"
#include <time.h>
#include <QLineF>
#include <time.h>

using namespace std;
// #include "ros/ros.h"
//#include <std_msgs/String.h>
//#include <geometry_msgs/PoseStamped.h>
// #include <agent_msg/msg/registered_comp_image.hpp>
// #include <px4_msgs/msg/piksi_pos_llh.hpp>

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    QApplication a(argc, argv);
    a.setApplicationName("QHAC3");
    a.setQuitOnLastWindowClosed(true);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    MainWidget w;
    w.show();


    QLineF l1 = QLineF(-44.583313, -343.290985, 822.619000, -271.690000);
    QLineF l2 = QLineF(-160.195480, 187.028183, 826.541000, -269.277000);
    QLineF l3 = QLineF(-40.472240, -347.337585, -160.195480, 187.028183);
    QLineF l4 = QLineF(-158.546341, 184.970642, 825.033000, -276.065000);
    QLineF l5 = QLineF(-368.875732, -206.243622, -40.630692, -343.096436);
    QLineF l6 = QLineF(-369.885254, -212.305832, -44.583313, -343.290985);
    QLineF l7 = QLineF(826.541000, -269.277000, -366.263275, -209.861633);
    QLineF l8 = QLineF(-158.546341, 184.970642, -40.472240, -347.337585);
    QLineF l9 = QLineF(-158.919342, 187.998703, -44.583313, -343.290985);
    QLineF l10 = QLineF(-40.630692, -343.096436, 822.619000, -271.690000);

//    (36.763471712, 127.28078951, 36.764115279, 127.29052471)
//    (36.768241909, 127.27947453, 36.764136976, 127.29056874)
//    (36.763435321, 127.28083566, 36.768241909, 127.27947453)
//    (36.768211159, 127.27950822, 36.764075932, 127.2905518)
//    (36.764704137, 127.27714898, 36.763473462, 127.28083388)
//    (36.764649618, 127.27713765, 36.763471712, 127.28078951)
//    (36.764136976, 127.29056874, 36.7646716, 127.27717831)
//    (36.768211159, 127.27950822, 36.763435321, 127.28083566)
//    (36.768265902, 127.2795088, 36.763471712, 127.28078951)
//    (36.763473462, 127.28083388, 36.764115279, 127.29052471)

//    QLineF l1 = QLineF(36.768494175, 127.29315909, 36.768211159, 127.27950822);
//    QLineF l2 = QLineF(36.7646716, 127.27717831, 36.763473462, 127.28083388);
//    QLineF l3 = QLineF(36.7646716, 127.27717831, 36.768265902, 127.2795088);
//    QLineF l4 = QLineF(36.763433944, 127.28079076, 36.768533852, 127.29313275);
//    QLineF l5 = QLineF(36.764075932, 127.2905518, 36.763433944, 127.28079076);
    cout << l1.length() << endl;
    vector<QLineF> dataset = {l1, l2, l3, l4, l5, l6, l7, l8, l9, l10};
    vector<int> id_set = {0, 1, 2, 11, 12};
    vector<float> EE_set = {0.8, 0.8, 1, 1, 1.2, 1, 1, 1, 1, 1};

    vector<int> lineset;
    SUVAlgo algo = SUVAlgo(dataset, EE_set);

//    duration = (double)(finish - start) / CLOCKS_PER_SEC;
//    cout << duration << endl;

//    vector<int> set = algo.choice_best_group(dataset, EE_set);
    clock_t start, finish;
    double duration;
    start = clock();
    vector<vector<int>> ans = algo.solution();
    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    printf("%f초", duration);
    cout << endl;
//    double INF = 987654321.0;
    vector <vector<double>> test_dataset = {
        {2.0, 3.0, 3.0},
        {3.0, 2.0, 3.0},
        {3.0, 3.0, 2.0}
    };


//    vector<int> assign = vector<int>();
//    HungarianAlgorithm ha = HungarianAlgorithm();
//    double cost = ha.Solve(test_dataset, assign);
//    vector<int> res = ha.SolveV2(test_dataset, assign);
//    cout << "WTF : " << cost << endl;
//    for (int idx: res) {
//        cout << "FUCK : " << idx << endl;
//    }
    return a.exec();
};
