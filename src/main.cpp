#include "controller.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include "ros/ros.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "Controller_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }

    ros::init(argc, argv, "gui_controller");
    Controller w;
    w.show();
    return a.exec();
}
