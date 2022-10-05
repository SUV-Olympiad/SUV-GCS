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

    return a.exec();
};
