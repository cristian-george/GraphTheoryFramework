#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QTimer>

#include "Graph.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void mouseReleaseEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;

private slots:
    void undirectedRadioButtonClicked();
    void directedRadioButtonClicked();

    void unweightedRadioButtonClicked();
    void weightedRadioButtonClicked();
    
    void shortPathTriggered();
    void topologicalSortTriggered();

    void connectedComponentsTriggered();
    void stronglyConnectedComponentsTriggered();

    void printAdjacencyMatrixTriggered();
    void printAdjacencyListsTriggered();

    void clearGraphTriggered();

private:
    void connectSlots();

    void drawEdges();
    void drawNodes();
    void drawShortPathBFS();
    void drawConnectedComponents();

    void drawNode(const Node& node, QColor color);
    void drawEdge(const Edge& edge, QColor color);

    QVector<QColor> randomColors(int count);

private:
    Ui::MainWindow *ui;
    QTimer *timer;

    bool isDrawingNode = false, isDrawingEdge = false;
    bool isShortPathRunning = false;
    bool isCCRunning = false;

    Graph graph;
    int firstNode = -1, lastNode = -1;

    std::vector<int> bfs_path;

    std::vector<ConnectedComponent> ccToColorize;
};
#endif // MAINWINDOW_H
