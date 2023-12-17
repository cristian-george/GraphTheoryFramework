#include <QMessageBox>

#include "MainWindow.h"

void MainWindow::shortPathTriggered()
{
    algorithm = EPathFinding::BFS;
    isColorizerRunning = true;
}

void MainWindow::topologicalSortTriggered()
{
    std::vector<int> topologicalSort = graph.TopologicalSort();

    if (topologicalSort.empty())
    {
        QMessageBox::information(this,
                                 "Topological sort",
                                 "The graph cannot be sorted in topological order. "
                                 "It contains a circuit! ");

        return;
    }

    // Determinăm coordonatele centrului ferestrei
    int xCenterScreen = size().width() / 2;
    int yCenterScreen = size().height() / 2;

    int numberOfNodes = topologicalSort.size();

    // Afişăm nodurile în ordinea corespunzătoare sortării topologice
    for (int i = 0; i < numberOfNodes; ++i)
    {
        int x = xCenterScreen + 100 * (i - numberOfNodes / 2);

        int node = topologicalSort[i] + 1;
        graph.MoveNode(node, x, yCenterScreen);
    }
}

void MainWindow::connectedComponentsTriggered()
{
    // Determinăm componentele conexe
    isColorizerRunning = true;
    ccToColorize = graph.ConnectedComponents();
    this->update();

    isColorizerRunning = false;
}

void MainWindow::stronglyConnectedComponentsTriggered()
{
    // Determinăm componentele tare conexe
    isColorizerRunning = true;
    ccToColorize = graph.StronglyConnectedComponents();
    this->update();

    isColorizerRunning = false;
}


void MainWindow::primTriggered()
{
    isColorizerRunning = true;
    // edgesToColorize = graph.Prim(0);
    this->update();

    isColorizerRunning = false;
}

void MainWindow::kruskalTriggered()
{
    isColorizerRunning = true;
    // edgesToColorize = graph.Kruskal();
    this->update();

    isColorizerRunning = false;
}


void MainWindow::dijkstraTriggered()
{
    // algorithm = EPathFinding::Dijkstra;
    isColorizerRunning = true;
}

void MainWindow::bellmanFordTriggered()
{
    // algorithm = EPathFinding::BellmanFord;
    isColorizerRunning = true;
}

void MainWindow::floydWarshallTriggered()
{
    // algorithm = EPathFinding::FloydWarshall;
    isColorizerRunning = true;
}
