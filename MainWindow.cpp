#include <QMessageBox>

#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->inputWeightLabel->setHidden(true);
    ui->inputWeightTextEdit->setHidden(true);

    ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
    ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode));

    connectSlots();
}


MainWindow::~MainWindow()
{
    delete timer;
    delete ui;
}


void MainWindow::mouseReleaseEvent(QMouseEvent* event)
{
    isDrawingNode = false;
    isDrawingEdge = false;

    if (!isShortPathRunning)
        bfs_path.clear();

    if (!isCCRunning)
        ccToColorize.clear();

    if (event->button() == Qt::RightButton)
    {
        // Reţinem poziţia click-ului de mouse în variabila "qpoint"
        QPoint qpoint = event->pos();

        // Apelăm metoda de adăugare a unui nod în graf
        // Nodul va fi afişat pe ecran la poziţia dată de "qpoint"
        isDrawingNode = graph.AddNode(qpoint.x(), qpoint.y());
    }
    else
        if (event->button() == Qt::LeftButton)
        {
            // Reţinem poziţia click-ului de mouse în variabila "qpoint"
            QPoint qpoint = event->pos();

            int foundNode = graph.GetNodeCloseTo(qpoint.x(), qpoint.y());

            if (foundNode == -1 && firstNode != -1)
            {
                // Mutăm nodul selectat la poziţia "qpoint"
                graph.MoveNode(firstNode, qpoint.x(), qpoint.y());

                firstNode = -1;

                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
                this->update();

                return;
            }

            if (firstNode == -1)
            {
                // Selecţia primului nod
                firstNode = foundNode;

                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
                this->update();

                return;
            }

            // Selecţia celui de-al doilea nod
            lastNode = foundNode;
            ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode));

            timer = new QTimer(this);
            timer->setSingleShot(true);
            timer->start(1000);

            connect(timer, &QTimer::timeout, this, [&]() {
                if (isShortPathRunning)
                {
                    /* Metoda BFS determină drumul de la "firstNode" la "lastNode"
                    / Atenţie! Indexarea se face de la 0. Din valorile etichetelor
                    / date ca parametru trebuie să scădem o unitate
                    */
                    bfs_path = graph.ShortPathBetween(firstNode - 1, lastNode - 1);
                    this->update();

                    if (bfs_path.empty())
                        QMessageBox::information(this, "BFS", "No path found!");

                    isShortPathRunning = false;
                }
                else
                {
                    // Adăugăm o muchie sau un arc între cele două noduri selectate
                    // Dacă graful este ponderat se va afişa şi valoarea "weight"
                    float weight = FLT_MAX;
                    if (graph.IsWeighted())
                        weight = ui->inputWeightTextEdit->toPlainText().toFloat();

                    isDrawingEdge = graph.AddEdge(firstNode, lastNode, weight);
                }

                // Resetăm cele două noduri selectate
                firstNode = -1;
                lastNode = -1;

                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
                ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode));

                this->update();
            });
        }
}


void MainWindow::paintEvent(QPaintEvent*)
{
    /* Edge based */
    drawEdges();
    drawShortPathBFS();

    /* Node based */
    drawNodes();
    drawConnectedComponents();

    this->update();
}


void MainWindow::undirectedRadioButtonClicked()
{
    graph.SetDirected(false);
    graph.Clear();
    bfs_path.clear();
    this->update();
}


void MainWindow::directedRadioButtonClicked()
{
    graph.SetDirected(true);
    graph.Clear();
    bfs_path.clear();
    this->update();
}


void MainWindow::unweightedRadioButtonClicked()
{
    graph.SetWeighted(false);
    graph.Clear();
    bfs_path.clear();

    ui->inputWeightLabel->setHidden(true);
    ui->inputWeightTextEdit->setHidden(true);
    this->update();
}


void MainWindow::weightedRadioButtonClicked()
{
    graph.SetWeighted(true);
    graph.Clear();
    bfs_path.clear();

    ui->inputWeightLabel->setHidden(false);
    ui->inputWeightTextEdit->setHidden(false);
    ui->inputWeightTextEdit->setText("0");
    this->update();
}


void MainWindow::shortPathTriggered()
{
    isShortPathRunning = true;
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
    isCCRunning = true;
    ccToColorize = graph.ConnectedComponents();
    this->update();

    isCCRunning = false;
}

void MainWindow::stronglyConnectedComponentsTriggered()
{
    // Determinăm componentele tare conexe
    isCCRunning = true;
    ccToColorize = graph.StronglyConnectedComponents();
    this->update();

    isCCRunning = false;
}


void MainWindow::printAdjacencyMatrixTriggered()
{
    graph.PrintAdjacencyMatrix();
}


void MainWindow::printAdjacencyListsTriggered()
{
    graph.PrintAdjacencyLists();
}


void MainWindow::clearGraphTriggered()
{
    graph.Clear();
    this->update();
}


void MainWindow::drawEdges()
{
    const std::vector<Edge>& edges = graph.GetUniqueEdges();

    if (edges.size())
    {
        for (const Edge& edge : edges)
        {
            drawEdge(edge, Qt::black);
        }

        if (isDrawingEdge == true)
        {
            drawEdge(edges[edges.size() - 1], Qt::cyan);
        }
    }
}


void MainWindow::drawNodes()
{
    const std::vector<Node>& nodes = graph.GetNodes();

    if (nodes.size())
    {
        for (const Node& node : nodes)
        {
            drawNode(node, Qt::white);
        }

        if (isDrawingNode == true)
        {
            drawNode(nodes[nodes.size() - 1], Qt::cyan);
        }
    }
}


void MainWindow::drawShortPathBFS()
{
    /* Afişăm cu o altă culoare nodurile şi muchiile/arcele
     * din componenţa drumului determinat cu algoritmul BFS
    */
    if (!bfs_path.empty())
    {
        for (size_t i = 0; i < bfs_path.size() - 1; ++i)
        {
            int node1 = bfs_path[i] + 1;
            int node2 = bfs_path[i + 1] + 1;
            drawEdge(graph.GetEdge(node1, node2), Qt::magenta);
        }

        for (size_t i = 0; i < bfs_path.size(); ++i)
        {
            int node = bfs_path[i] + 1;
            drawNode(graph.GetNode(node), Qt::magenta);
        }
    }

    /* Colorăm cu verde nodul de plecare selectat
     * Colorăm cu roşu nodul de sosire selectat
    */
    if (isShortPathRunning)
    {
        if (firstNode != -1)
            drawNode(graph.GetNode(firstNode), Qt::green);
        if (lastNode != -1)
            drawNode(graph.GetNode(lastNode), Qt::red);
    }
}


void MainWindow::drawConnectedComponents()
{
    if (!ccToColorize.empty())
    {
        int count = ccToColorize.size();

        auto colors = randomColors(count);

        for (size_t i = 0; i < ccToColorize.size(); ++i)
        {
            // Determin componenta i şi extrag nodurile din aceasta
            auto nodes = ccToColorize[i].GetNodes();
            for (size_t j = 0; j < nodes.size(); ++j)
            {
                int node = nodes[j];
                drawNode(graph.GetNode(node), colors[i]);
            }
        }
    }
}
