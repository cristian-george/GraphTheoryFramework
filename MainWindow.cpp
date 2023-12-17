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
    if (timer != nullptr)
        delete timer;
    delete ui;
}


void MainWindow::mouseReleaseEvent(QMouseEvent* event)
{
    isDrawingNode = false;
    isDrawingEdge = false;

    if (!isColorizerRunning)
    {
        removeColor();
    }

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
            QPoint qpoint = event->pos(); // Reţinem poziţia click-ului de mouse în variabila "qpoint"

            int foundNode = graph.GetNodeCloseTo(qpoint.x(), qpoint.y());

            // Mutăm nodul selectat la poziţia "qpoint"
            if (foundNode == -1 && firstNode != -1)
            {
                graph.MoveNode(firstNode, qpoint.x(), qpoint.y());

                firstNode = -1;

                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
                this->update();

                return;
            }

            // Selecţia primului nod
            if (firstNode == -1)
            {
                firstNode = foundNode;

                ui->firstNodeLabel->setText("First node: " + QString::number(firstNode));
                this->update();

                return;
            }

            // Selecţia celui de-al doilea nod
            lastNode = foundNode;
            ui->lastNodeLabel->setText("Last node: " + QString::number(lastNode));

            applyChanges();
        }
}


void MainWindow::paintEvent(QPaintEvent*)
{
    /* Edge based */
    drawEdges();
    colorizeEdges();

    /* Node based */
    drawNodes();
    colorizeNodes();
    colorizeConnectedComponents();

    this->update();
}


void MainWindow::applyChanges()
{
    timer = new QTimer(this);
    timer->setSingleShot(true);
    timer->start(1000);

    connect(timer, &QTimer::timeout, this, [&]() {
        if (algorithm != EPathFinding::None)
        {
            try
            {
                edgesToColorize = graph.GetPathBetween(firstNode - 1,
                                                       lastNode - 1,
                                                       algorithm);
            } catch (const char* e)
            {
                QMessageBox::information(this, "Path finding", e);
            }

            this->update();

            // Resetăm algoritmul selectat
            isColorizerRunning = false;
            algorithm = EPathFinding::None;
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


void MainWindow::removeColor()
{
    edgesToColorize.clear();
    ccToColorize.clear();

    algorithm = EPathFinding::None;
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


void MainWindow::colorizeNodes()
{
    /* Colorăm cu verde nodul de plecare selectat
     * Colorăm cu roşu nodul de sosire selectat
    */
    if (algorithm != EPathFinding::None)
    {
        if (firstNode != -1)
            drawNode(graph.GetNode(firstNode), Qt::green);
        if (lastNode != -1)
            drawNode(graph.GetNode(lastNode), Qt::red);
    }
}

void MainWindow::colorizeEdges()
{
    /* Afişăm cu o altă culoare muchiile din componenţa
     * arborelui parţial de cost minim / drumului de cost minim
    */
    if (!edgesToColorize.empty())
    {
        for (size_t i = 0; i < edgesToColorize.size(); ++i)
        {
            drawEdge(edgesToColorize[i], Qt::magenta);
        }
    }
}

void MainWindow::colorizeConnectedComponents()
{
    if (!ccToColorize.empty())
    {
        int count = ccToColorize.size();

        const std::vector<QColor> colors = randomColors(count);

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
