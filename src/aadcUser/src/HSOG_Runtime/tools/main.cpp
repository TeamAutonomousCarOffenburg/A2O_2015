#include <iostream>

#include "world/WorldViewer.h"

#include <qt4/Qt/qapplication.h>
#include <qt4/Qt/qwidget.h>

using namespace std;

int main (int argc, char**argv)
{
  QApplication application(argc, argv);

  WorldViewer viewer;

  viewer.setWindowTitle("A2O World Viewer");

  viewer.show();

  return application.exec();
}
