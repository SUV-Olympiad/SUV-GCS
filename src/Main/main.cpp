//#include <QtGui/QApplication>
#include "mainwindow.h"
#include <QApplication>
#include <stdio.h> 
#include "manager.h"
#include <GL/glut.h>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp> //임시


#include "swalgo.h"
#include "hungarian.h"
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

    MainWindow w;
    w.show();


    QLineF l1 = QLineF(-26, 32, -82, -12);
    QLineF l2 = QLineF(-85, 96, 60, 59);
    QLineF l3 = QLineF(68, 23, 85, 90);
    QLineF l4 = QLineF(41, -39, -29, 100);
    QLineF l5 = QLineF(-16, -11, -78, 62);
    vector<QLineF> dataset = {l1, l2, l3, l4, l5};
    vector<int> id_set = {0, 1, 2, 11, 12};
    vector<float> EE_set = {0.8, 0.8, 1, 1, 1.2};
    vector<bool> visited(5);
    vector<bool> dp(5);
    vector<int> lineset;
    SWAlgo algo = SWAlgo();

    clock_t start, finish;
    double duration;

    start = clock();
    algo.grouping(0, dataset, lineset, visited, dp);
    finish = clock();

    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    cout << duration << endl;
    algo.print_grouping(id_set);

    vector<int> set = algo.choice_best_group(dataset, EE_set);
    for (int s: set) {
        cout << s << " ";
    }
    cout << endl;
    double INF = 987654321.0;
    vector <vector<double>> test_dataset = {
        {2.0, 3.0, 3.0},
        {3.0, 2.0, 3.0},
        {3.0, 3.0, 2.0}
    };


    vector<int> assign = vector<int>();
    HungarianAlgorithm ha = HungarianAlgorithm();
//    double cost = ha.Solve(test_dataset, assign);
    vector<int> res = ha.SolveV2(test_dataset, assign);
//    cout << "WTF : " << cost << endl;
    for (int idx: res) {
        cout << "FUCK : " << idx << endl;
    }
    return a.exec();
};
