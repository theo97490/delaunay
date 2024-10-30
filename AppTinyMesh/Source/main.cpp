#include "qte.h"
#include "tp_geom/delaunay.h"
#include <QtWidgets/qapplication.h>
#include <libdelaunay.rs.h>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	MainWindow mainWin;
	mainWin.showMaximized();

	// const char* aa = "yolo salut comment ça va le crustacée ?";
	// int* bruh = array_test(aa);
	// bruh[0] = 42;
	// array_test(aa);
	//
	// TestStruct a = RustTypes::make_TestStruct(10);
	// a.get_value();

	// const auto drawable = LibDelaunay::compute_triangulation_2d("");
	// Triangulation2D tri;
	// tri.add_point(Vector{0.,0.,0.});
	// tri.add_point(Vector{5.,5.,0.});
	// tri.add_point(Vector{5.,0.,0.});
	// tri.add_point(Vector{0.,5.,0.});

	

	return app.exec();

}

/**
*  RENDU:
*  Readme + rapport rapide avec screenshots
**/
