#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

int main(int argc, char* argv[])
{
	Display* disp = new Display(1000, 800, "Wellcome");
	Renderer renderer;
	igl::opengl::glfw::Viewer viewer(4);
	std::ifstream file("configuration.txt");
	if (file.is_open()) {
		std::string line;
		bool sphere = false;
		while (getline(file, line)) {
			if (sphere) {
				viewer.load_mesh_from_file(line);
				viewer.data().MyTranslate(Eigen:: Vector3f(5, 0, 0));
			}
			else {
				viewer.length = 1.6;
				for (int i = 0; i < viewer.num_of_cyl; i++) {
					viewer.load_mesh_from_file(line);
					viewer.pos_cylinder();
				}
				viewer.MyTranslate(Eigen::Vector3f(0, -2, -9));
				viewer.data(0).MyTranslate(Eigen::Vector3f(0, -1.6, 0));
			}
			sphere = true;
			
		}
		for (auto& mesh : viewer.data_list) {
			if (mesh.id != viewer.num_of_cyl) {
				viewer.data(mesh.id).set_face_based(!viewer.data(mesh.id).face_based);
			}
			renderer.core(mesh.id).toggle(viewer.data(mesh.id).show_lines);
			
		}
		
		file.close();
	}
	else {
		std::cout << " config wasn't open " << std::endl;
	}
	
	
	//viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/sphere.obj");
	//viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/cube.obj");
	//viewer.load_mesh_from_file("C:/Dev/EngineIGLnew/tutorial/data/bunny.off");
	Init(*disp);
	renderer.init(&viewer);
	disp->SetRenderer(&renderer);
	disp->launch_rendering(true);

	delete disp;
}