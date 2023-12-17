#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QTimer>

#include "Graph.h"
#include "Enums.h"

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

    void primTriggered();
    void kruskalTriggered();

    void dijkstraTriggered();
    void bellmanFordTriggered();
    void floydWarshallTriggered();

    void printAdjacencyMatrixTriggered();
    void printAdjacencyListsTriggered();

    void clearGraphTriggered();

private:
    void connectSlots();

private:
    // Metodă care tratează 1. adăugarea muchiilor în graf
    //                      2. lansarea algoritmilor BFS, Dijkstra, Bellman-Ford, Floyd-Warshall
    void applyChanges();

private:
    void drawEdges();
    void drawNodes();

    void colorizeNodes();
    void colorizeEdges();
    void colorizeConnectedComponents();
    void removeColor();

    void drawNode(const Node& node, QColor color);
    void drawEdge(const Edge& edge, QColor color);

    std::vector<QColor> randomColors(int count);

private:
    Ui::MainWindow *ui;
    QTimer *timer = nullptr;

    bool isDrawingNode = false, isDrawingEdge = false;
    bool isColorizerRunning = false;

    std::vector<Edge> edgesToColorize;
    std::vector<ConnectedComponent> ccToColorize;

private:
    Graph graph;

    int firstNode = -1;
    int lastNode = -1;

    EPathFinding algorithm = EPathFinding::None;
};
#endif // MAINWINDOW_H
