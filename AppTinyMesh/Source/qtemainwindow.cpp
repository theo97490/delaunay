#include "color.h"
#include "libdelaunay.rs.h"
#include "mathematics.h"
#include "mesh.h"
#include "meshcolor.h"
#include "qte.h"
#include "realtime.h"
#include "tp_geom/delaunay.h"
#include "ui_interface.h"
#include <cstdlib>
#include <memory>
#include <qobjectdefs.h>
#include "tp_geom/pointcloud.h"
#include "tp_geom/off.h"

MainWindow::MainWindow() : QMainWindow(), uiw(new Ui::Assets)
{
	// Chargement de l'interface
    uiw->setupUi(this);

	// Chargement du GLWidget
	meshWidget = new MeshWidget;
	QGridLayout* GLlayout = new QGridLayout;
	GLlayout->addWidget(meshWidget, 0, 0);
	GLlayout->setContentsMargins(0, 0, 0, 0);
    uiw->widget_GL->setLayout(GLlayout);

	// Creation des connect
	CreateActions();

	meshWidget->SetCamera(Camera(Vector(10, 0, 0), Vector(0.0, 0.0, 0.0)));
}

MainWindow::~MainWindow()
{
	delete meshWidget;
}

void MainWindow::CreateActions()
{
	// Buttons
	  connect(uiw->example, SIGNAL(clicked()), this, SLOT(loadExample()));
	  connect(uiw->tri_load, SIGNAL(clicked()), this, SLOT(loadTriangulation()));
	  connect(uiw->tri_load_delaunay, SIGNAL(clicked()), this, SLOT(loadNaiveToDelaunay()));
	  connect(uiw->tri_next, SIGNAL(clicked()), this, SLOT(triangulationNext()));
    connect(uiw->resetcameraButton, SIGNAL(clicked()), this, SLOT(ResetCamera()));
    connect(uiw->wireframe, SIGNAL(clicked()), this, SLOT(UpdateMaterial()));

    connect(uiw->normal2, SIGNAL(clicked()), this, SLOT(UpdateColor()));
    connect(uiw->curvature, SIGNAL(clicked()), this, SLOT(UpdateColor()));
    connect(uiw->heat, SIGNAL(clicked()), this, SLOT(UpdateColor()));

    connect(uiw->heatReset, SIGNAL(clicked()), this, SLOT(InitHeat()));
    connect(uiw->heatSimulate, SIGNAL(clicked()), this, SLOT(SimulateHeat()));
    connect(uiw->radioShadingButton_1, SIGNAL(clicked()), this, SLOT(UpdateMaterial()));

	// Widget edition
	connect(meshWidget, SIGNAL(_signalUpdate()), this, SLOT(SimulateHeat()));
	connect(meshWidget, SIGNAL(_signalEditSceneLeft(const Ray&)), this, SLOT(editingSceneLeft(const Ray&)));
	connect(meshWidget, SIGNAL(_signalEditSceneRight(const Ray&)), this, SLOT(editingSceneRight(const Ray&)));
}

void MainWindow::editingSceneLeft(const Ray&)
{
}

void MainWindow::editingSceneRight(const Ray&)
{
}

Vector toVec(LibDelaunay::Vec3 vec) {
  return Vector(vec.v[0], vec.v[1], vec.v[2]);
}

MeshColor convertToQtMesh(const TriangleMesh& triMesh){
	std::vector<Vector> normals;
	std::vector<int> indices, normals_indices;

	size_t n = 0;
	for (auto& tri : triMesh.triangles){
    indices.push_back(tri.vertices[0]);
    indices.push_back(tri.vertices[1]);
	  indices.push_back(tri.vertices[2]);

		auto n0 = triMesh.vertices[tri.vertices[0]];
		auto n1 = triMesh.vertices[tri.vertices[1]];
		auto n2 = triMesh.vertices[tri.vertices[2]];

		auto normal = Normalized((n1 - n0) / (n2 - n0));
		normals.push_back(normal);

		normals_indices.push_back(n);
		normals_indices.push_back(n);
		normals_indices.push_back(n++);
	}

	return MeshColor(Mesh(triMesh.vertices, normals, indices, normals_indices));
}

MeshColor convertToQtMesh(const LibDelaunay::DrawableMesh& triMesh){
	std::vector<Vector> normals;
  std::vector<Vector> vertices;
	std::vector<int> indices, normals_indices;

  for (int i = 0; i < triMesh.vertices.size(); i++){
      vertices.emplace_back(toVec(triMesh.vertices[i]));
  }

	size_t n = 0;
	for (auto& tri : triMesh.triangles){
    indices.push_back(tri.vertices[0]);
    indices.push_back(tri.vertices[1]);
	  indices.push_back(tri.vertices[2]);

		auto n0 = toVec(triMesh.vertices[tri.vertices[0]]);
		auto n1 = toVec(triMesh.vertices[tri.vertices[1]]);
		auto n2 = toVec(triMesh.vertices[tri.vertices[2]]);

		auto normal = Normalized((n1 - n0) / (n2 - n0));
		normals.push_back(normal);

		normals_indices.push_back(n);
		normals_indices.push_back(n);
		normals_indices.push_back(n++);
	}

	return MeshColor(Mesh(vertices, normals, indices, normals_indices));
}

void MainWindow::loadQueen()
{
	mode = View::TRIANGULATION;
	queen = load_off("queen.off");
  meshColor = convertToQtMesh(queen);
	UpdateGeometry();
}

void MainWindow::triangulationNext() {
    if (mode != View::TRIANGULATION or last >= loaded.size()) return;

    Vector p = loaded[last++] / 1000;

    if (!uiw->tri_elevate->isChecked())
      p[2] = 0;

    del->add_point(p);

    meshColor = convertToQtMesh(del->get_mesh());
    UpdateGeometry();
}

void MainWindow::loadNaiveToDelaunay() {
	mode = View::TRIANGULATION;
  const auto points = load_point_cloud(uiw->tri_path->text().toStdString());
  Triangulation2D tri;

  for (size_t i = 0; i < points.size(); i++){
    Vector p = points[i] / 1000;

    if (!uiw->tri_elevate->isChecked())
      p[2] = 0;

    tri.add_point(p);
  }

  const auto mesh = TriMeshAlgorithm::to_delaunay(tri);

  meshColor = convertToQtMesh(mesh);
  UpdateGeometry();
}

void MainWindow::loadTriangulation() {
  const auto path = uiw->tri_path->text().toStdString();
  const auto elevate = uiw->tri_elevate->isChecked();
  const auto mesh = LibDelaunay::compute_triangulation_2d(path, 0.001, elevate);
  meshColor = convertToQtMesh(mesh);
  UpdateGeometry();

	return; 
	mode = View::TRIANGULATION;

  if (uiw->tri_delaunay->isChecked())
    del = std::make_unique<DelaunayTriangulation2D>();
  else
    del = std::make_unique<Triangulation2D>();

  loaded = load_point_cloud(uiw->tri_path->text().toStdString());
  
  const size_t size = uiw->tri_load_first->isChecked() ? 3 : loaded.size();

  for (last = 0; last < size; last++){
    Vector p = loaded[last] / 1000;

    if (!uiw->tri_elevate->isChecked())
      p[2] = 0;

    del->add_point(p);
  }

  meshColor = convertToQtMesh(del->get_mesh());
	UpdateGeometry();
}

void MainWindow::LaplacianCurvature(){
  if (mode != View::QUEEN or queen.is_empty()) return;

	std::vector<Color> cols;
	cols.resize(meshColor.Vertexes());

	std::vector<double> laplacians[3];
   for (size_t i = 0; i < cols.size(); i++){
     double c0 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][0];});
     double c1 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][1];});
     double c2 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][2];});
     laplacians[0].push_back(c0);
     laplacians[1].push_back(c1);
     laplacians[2].push_back(c2);
   }

   for (size_t i = 0; i < cols.size(); i++){
    auto normal = Vector{laplacians[0][i], laplacians[1][i], laplacians[2][i]};
    auto H = Norm(normal)/2;

    H = double(H)/15000;

    cols[i] = Color(std::max(0.,H), 0., 0., 1.) ;
   }

   meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
}

void MainWindow::LaplacianNormals(){
  if (mode != View::QUEEN or queen.is_empty()) return;

	std::vector<Color> cols;
	cols.resize(meshColor.Vertexes());

	std::vector<double> laplacians[3];
   for (size_t i = 0; i < cols.size(); i++){
     double c0 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][0];});
     double c1 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][1];});
     double c2 = queen.laplacian(i, [](TriangleMesh& m, size_t v){ return m.vertices[v][2];});
     laplacians[0].push_back(c0);
     laplacians[1].push_back(c1);
     laplacians[2].push_back(c2);
   }

   for (size_t i = 0; i < cols.size(); i++){
     auto normal = Vector{laplacians[0][i], laplacians[1][i], laplacians[2][i]};
     // auto H = Norm(a)/2;
     // H = double(H)/15000;
     auto vcolor = ((Normalized(normal) + Vector(1,1,1))/2);
     cols[i] = Color(vcolor[0] , vcolor[1], vcolor[2], 1.) ;
   }

   meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
}

void MainWindow::InitHeat(){
  if (mode != View::QUEEN or queen.is_empty()) return;

	std::vector<Color> cols;
	cols.resize(meshColor.Vertexes());
	heat.clear();
	heat.resize(meshColor.Vertexes(), 0);
	heat_pos.clear();

	// Create 5 random patch of heat inside the mesh
	for (int i = 0; i < 1; i++){
	  //rand pos
	  int p = rand() % meshColor.Vertexes();
		heat_pos.push_back(p);
		heat[i] = 1.;
	};


	for (size_t i = 0; i < meshColor.Vertexes(); i++){
	  cols[i] = Color(heat[i],0.,0.,1.);
	}

  meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
 	UpdateGeometry();
	UpdateMaterial();
}

//// Version qui crée plusieurs patchs de chaleurs qui vont se dissiper dans le mesh (chaleur non constante)

// void MainWindow::InitHeat(){
//   if (triMesh.is_empty()) return;

// 	std::vector<Color> cols;
// 	cols.resize(meshColor.Vertexes());
// 	heat.clear();
// 	heat.resize(meshColor.Vertexes(), 0);

// 	// Create 5 random patch of heat inside the mesh
// 	for (int i = 0; i < 30; i++){
// 	  //rand pos
// 	  int p = rand() % meshColor.Vertexes();

// 		std::unordered_set<MTriangle*> previous, found, checked;
// 		for (auto& f : triMesh.faces_around_v(p))
// 		  previous.insert(f);

// 		const auto explore_face = [&](MTriangle* tri, double heat){
// 		  for (int i = 0; i < 3; i++){
// 				size_t v_id = tri->vertices[i];

//   		  for (auto& face : triMesh.faces_around_v(v_id)){
//       		if (checked.count(face)) continue;
//           found.insert(face);
//   			}

//         double& v_heat = this->heat[v_id];
//         if (v_heat < heat) v_heat = heat;
// 			}
// 		};

// 		// Explore faces of the current set
// 		const int k = 10;
// 		for (double z = k; z > 0; z--){
// 		  for (auto& f : previous){
// 				explore_face(f, z/k);
// 			}

// 			checked.insert(previous.begin(), previous.end());
// 			previous = found;
// 			found.clear();
// 		}
// 	}

// 	for (size_t i = 0; i < meshColor.Vertexes(); i++){
// 	  cols[i] = Color(heat[i],0.,0.,1.);
// 	}

//   meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
//  	UpdateGeometry();
// 	UpdateMaterial();
// }
void MainWindow::SimulateHeat(){
  if (mode != View::QUEEN or queen.is_empty() || !uiw->heat->isChecked() || heat.empty()) return;

  const double delta = 0.000001;
 	std::vector<Color> cols;
	std::vector<double> prev_heat = heat; // Pas le mieux mais sert à tester la configuration
	cols.resize(meshColor.Vertexes());

	double m = 0.0;
	for (size_t i = 0; i < cols.size(); i++){
		double l = queen.laplacian(i, [&](TriangleMesh& m, size_t v){
			return prev_heat[v];
		});

		heat[i] = prev_heat[i] + delta * l;
		m = std::max(m, heat[i]);
	}

	for (auto p : heat_pos)
	  heat[p] = 1.0;

	for (size_t i = 0; i < cols.size(); i++){
		cols[i] = Color(heat[i]/m, 0., 0., 1.);
	}
	meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
	UpdateGeometry();
	UpdateMaterial();
}

//// Version qui crée plusieurs patchs de chaleurs qui vont se dissiper dans le mesh (chaleur non constante)

// void MainWindow::SimulateHeat(){
//   if (triMesh.is_empty() || !uiw->heat->isChecked() || heat.empty()) return;

//   const double delta = 0.01;
//  	std::vector<Color> cols;
// 	std::vector<double> prev_heat = heat; // Pas le mieux mais sert à tester la configuration
// 	cols.resize(meshColor.Vertexes());

// 	double m = 0.0;
// 	for (size_t i = 0; i < cols.size(); i++){
// 	  double l = triMesh.laplacian(i, [&](TriangleMesh& m, size_t v){
// 			return prev_heat[v];
// 		});

// 		heat[i] = std::max(0., prev_heat[i] + delta * l);
// 		m = std::max(m, heat[i]);
// 	}

// 	for (size_t i = 0; i < cols.size(); i++){
// 		cols[i] = Color(heat[i]/m, 0., 0., 1.);
// 	}
// 	meshColor = MeshColor(meshColor, cols, meshColor.VertexIndexes());
// 	UpdateGeometry();
// 	UpdateMaterial();
// }

void MainWindow::UpdateGeometry()
{
	meshWidget->ClearAll();
	meshWidget->AddMesh("default", meshColor);

    uiw->lineEdit->setText(QString::number(meshColor.Vertexes()));
    uiw->lineEdit_2->setText(QString::number(meshColor.Triangles()));

	UpdateMaterial();
}

void MainWindow::UpdateMaterial()
{
    meshWidget->UseWireframeGlobal(uiw->wireframe->isChecked());

    if (uiw->radioShadingButton_1->isChecked())
		meshWidget->SetMaterialGlobal(MeshMaterial::Normal);
		else
		meshWidget->SetMaterialGlobal(MeshMaterial::Color);
}

void MainWindow::UpdateColor()
{
  if (uiw->normal2->isChecked())
    LaplacianNormals();
  else if (uiw->curvature->isChecked())
    LaplacianCurvature();
  else if (uiw->heat->isChecked())
    InitHeat();

  UpdateGeometry();
  UpdateMaterial();
}

void MainWindow::ResetCamera()
{
	meshWidget->SetCamera(Camera(Vector(-10.0), Vector(0.0)));
}

void MainWindow::SetMesh(const MeshColor& mesh){
  meshColor = mesh;
  UpdateGeometry();
  UpdateMaterial();
}
