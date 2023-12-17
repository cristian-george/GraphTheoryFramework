#include "MainWindow.h"
#include "ui_MainWindow.h"

void MainWindow::undirectedRadioButtonClicked()
{
    graph.SetDirected(false);
    graph.Clear();
    
    removeColor();

    this->update();
}

void MainWindow::directedRadioButtonClicked()
{
    graph.SetDirected(true);
    graph.Clear();
    
    removeColor();

    this->update();
}

void MainWindow::unweightedRadioButtonClicked()
{
    graph.SetWeighted(false);
    graph.Clear();
    
    removeColor();

    ui->inputWeightLabel->setHidden(true);
    ui->inputWeightTextEdit->setHidden(true);
    this->update();
}

void MainWindow::weightedRadioButtonClicked()
{
    graph.SetWeighted(true);
    graph.Clear();
    
    removeColor();

    ui->inputWeightLabel->setHidden(false);
    ui->inputWeightTextEdit->setHidden(false);
    ui->inputWeightTextEdit->setText("0");
    this->update();
}
