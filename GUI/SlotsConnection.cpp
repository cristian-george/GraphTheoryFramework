#include "MainWindow.h"
#include "ui_MainWindow.h"

void MainWindow::connectSlots()
{
    connect(
        ui->undirectedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::undirectedRadioButtonClicked);

    connect(
        ui->directedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::directedRadioButtonClicked);

    connect(
        ui->unweightedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::unweightedRadioButtonClicked);

    connect(
        ui->weightedRadioButton, &QRadioButton::clicked,
        this, &MainWindow::weightedRadioButtonClicked);


    connect(
        ui->actionPrintAdjacencyMatrix, &QAction::triggered,
        this, &MainWindow::printAdjacencyMatrixTriggered);

    connect(
        ui->actionPrintAdjacencyLists, &QAction::triggered,
        this, &MainWindow::printAdjacencyListsTriggered);


    connect(
        ui->actionShortPath, &QAction::triggered,
        this, &MainWindow::shortPathTriggered);

    connect(
        ui->actionTopologicalSort, &QAction::triggered,
        this, &MainWindow::topologicalSortTriggered);


    connect(
        ui->actionConnectedComponents, &QAction::triggered,
        this, &MainWindow::connectedComponentsTriggered);

    connect(
        ui->actionStronglyConnectedComponents, &QAction::triggered,
        this, &MainWindow::stronglyConnectedComponentsTriggered);


    connect(
        ui->actionClearGraph, &QAction::triggered,
        this, &MainWindow::clearGraphTriggered);
}
