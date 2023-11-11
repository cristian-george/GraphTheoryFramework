#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QTimer>

#include "Graph.h"
#include "Node.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void connectSlots();

    void mouseReleaseEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent* event);

    void drawNode(const Node& node, Qt::GlobalColor color);
    void drawEdge(const Edge& edge, Qt::GlobalColor color);

    void addNode(QPoint point);
    void moveNode(QPoint point);
    void addEdge();

private slots:
    void undirectedRadioButtonClicked();
    void directedRadioButtonClicked();

    void unweightedRadioButtonClicked();
    void weightedRadioButtonClicked();

private:
    Ui::MainWindow *ui;
    QTimer *timer;

    Graph graph;
    Node firstNode, lastNode;
    bool isDrawingNode, isDrawingEdge;
};
#endif // MAINWINDOW_H
