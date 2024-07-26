#include <igcclib/geometry/igcclib_obj_eigen.hpp>
#include "./../plot/mesh_viewer.h"

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>
#include <map>

namespace mine = _NS_UTILITY;

int main(int argc, char* argv[])
{
	const std::string fn_model = "e:/data/scan/male/M0001/output/deform_no_shading_no_nose.obj";
	const std::string fn_texture = "e:/data/scan/male/M0001/output/deform_no_shading_no_nose.png";

	auto app = mine::MeshViewer::init();

	mine::TriangularMesh mesh;
	mine::load_obj(fn_model, mesh, fn_texture);

	//rotate the head so that it faces the front
	auto tmat_rotmat = mine::rotation_matrix(-std::acos(-1) / 2, { 1,0,0 });		
	mesh.set_transmat(tmat_rotmat);

	auto& img = mesh.get_texture_image();

	app->set_mesh("my_mesh", mesh);
	app->set_view_range_tight();
	app->set_window_size(600);
	//app.set_visible(false);

	cv::Mat cv_image;
	app->render_image_RGBA(cv_image);
	cv::cvtColor(cv_image, cv_image, cv::COLOR_RGBA2BGRA);
	cv::imwrite("d:/test.png", cv_image);

	app->set_visible(true);
	app->exec();

	return 0;
}