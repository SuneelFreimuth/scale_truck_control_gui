#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <thread>
#include <vector>

#include <QMainWindow>
#include <QCloseEvent>

#include "ros/ros.h"

#include <opencv2/opencv.hpp>

QT_BEGIN_NAMESPACE
namespace Ui { class Controller; }
QT_END_NAMESPACE

enum TruckIndex {
    LV,
    FV1,
    FV2
};

constexpr size_t NUM_TRUCKS = 3;

class Controller : public QMainWindow
{
    Q_OBJECT

public:
    Controller(QWidget *parent = nullptr);
    ~Controller();

    int MinVel;
    int MaxVel;
    int MinDist;
    int MaxDist;
    int FV1_cf;
    int FV2_cf;

protected:
    void closeEvent(QCloseEvent* event);

private slots:
    void on_MVelSlider_valueChanged(int value);
    void on_MDistSlider_valueChanged(int value);

    void on_LVVelSlider_valueChanged(int value);
    void on_LVDistSlider_valueChanged(int value);

    void on_FV1VelSlider_valueChanged(int value);
    void on_FV1DistSlider_valueChanged(int value);

    void on_FV2VelSlider_valueChanged(int value);
    void on_FV2DistSlider_valueChanged(int value);

    void on_pushButton_clicked();

    // cv::Mat display_Map(UDPsock::UDP_DATA value);

    void on_LVBox_activated(int index);
    void on_FV1Box_activated(int index);
    void on_FV2Box_activated(int index);

    void on_Send_clicked();

    void on_FV1_cf_toggled(bool checked);
    void on_FV2_cf_toggled(bool checked);

private:
    Ui::Controller *ui;

    ros::NodeHandle node;
    std::array<ros::Publisher, NUM_TRUCKS> target_vel_pubs;
    std::array<ros::Publisher, NUM_TRUCKS> target_dist_pubs;
    std::thread ros_spin;

    void publishVelocity(TruckIndex truck, float target_vel);
    void publishDistance(TruckIndex truck, float target_dist);
};

#endif // CONTROLLER_H
