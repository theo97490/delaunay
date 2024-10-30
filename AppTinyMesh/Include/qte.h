#ifndef __Qte__
#define __Qte__

#include <QtWidgets/qmainwindow.h>
#include "realtime.h"
#include "meshcolor.h"
#include <tp_geom/triangulation.h>
#include <tp_geom/delaunay.h>

QT_BEGIN_NAMESPACE
	namespace Ui { class Assets; }
QT_END_NAMESPACE

enum class View {
  QUEEN,
  TRIANGULATION
};

class MainWindow : public QMainWindow
{
  Q_OBJECT
private:
  Ui::Assets* uiw;           //!< Interface

  MeshWidget* meshWidget;   //!< Viewer
  MeshColor meshColor;		//!< Mesh.
  TriangleMesh queen; 
  std::vector<double> heat;
  std::vector<size_t> heat_pos;
  std::unique_ptr<Triangulation2D> del;
  size_t last = 0;
  std::vector<Vector> loaded;
  View mode = View::QUEEN;
public:
  MainWindow();
  ~MainWindow();
  void CreateActions();
  void UpdateGeometry();
  void SetMesh(const MeshColor& mesh);

public slots:
  void editingSceneLeft(const Ray&);
  void editingSceneRight(const Ray&);
  void loadQueen();
  void loadTriangulation();
  void loadNaiveToDelaunay();
  void triangulationNext();
  void ResetCamera();
  void UpdateMaterial();
  void UpdateColor();

  void LaplacianNormals();
  void LaplacianCurvature();

  void InitHeat();
  void SimulateHeat();
};

#endif
