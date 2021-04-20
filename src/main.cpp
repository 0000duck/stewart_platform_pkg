#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QQmlContext>
#include <QTimer>
#include <QCoreApplication>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executor.hpp>

#include "hmi_node1.h"
#include "qt_executor.h"

using namespace rclcpp;

int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto server = std::make_shared<HmiNode>();

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("oscillator", &*server);

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    engine.load(url);

    QtExecutor executor;
    
    executor.add_node(server);
    
    executor.start();

    auto res = app.exec();
    printf("Exited QT thread\n");
    rclcpp::shutdown();

    return res;
}

