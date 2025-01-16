#include <CGAL/Aff_transformation_3.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/transform.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/IO/OBJ.h>
#include <CGAL/minkowski_sum_3.h>
#include <CGAL/polygon_mesh_processing.h>
#include <iostream>
#include <fstream>

using Kernel         = CGAL::Exact_predicates_exact_constructions_kernel;
using Polyhedron3    = CGAL::Polyhedron_3<Kernel>;
using NefPolyhedron3 = CGAL::Nef_polyhedron_3<Kernel>;
using Affine         = CGAL::Aff_transformation_3<Kernel>;

bool verbose = true;

int main() {
    // Read the box mesh
    std::string   box_filename = "../data/obj/FallingBoxPerpendicular.obj";
    std::ifstream box_file(box_filename);
    Polyhedron3   box_mesh;
    if (box_file.fail() || !CGAL::IO::read_OBJ(box_file, box_mesh)) {
        std::cerr << "Failed to open file: " << box_filename << std::endl;
        return 1;
    }

    // Transform the box mesh
    Affine trans_box(1, 0, 0, 0.25, 0, 1, 0, 0.25, 0, 0, 1, 0);
    CGAL::Polygon_mesh_processing::transform(trans_box, box_mesh);

    // Read the ground mesh
    std::string   ground_filename = "../data/obj/Ground.obj";
    std::ifstream ground_file(ground_filename);
    Polyhedron3   ground_mesh;
    if (ground_file.fail() || !CGAL::IO::read_OBJ(ground_file, ground_mesh)) {
        std::cerr << "Failed to open file: " << ground_filename << std::endl;
        return 1;
    }

    // Convert the Polyhedron3 to NefPolyhedron3
    NefPolyhedron3 box_nef(box_mesh);
    NefPolyhedron3 ground_nef(ground_mesh);

    // Compute the Minkowski sum
    NefPolyhedron3 minkowski_sum = CGAL::minkowski_sum_3(box_nef, ground_nef);

    // Minkowski difference (poly1 ⊖ poly2 = poly1 ⊕ (-poly2))
    Affine neg_trans(-1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0);
    CGAL::Polygon_mesh_processing::transform(neg_trans, ground_mesh);
    NefPolyhedron3 minkowski_diff = CGAL::minkowski_sum_3(box_nef, ground_nef);

    // Convert the NefPolyhedron3 to Polyhedron3
    Polyhedron3 minkowski_sum_res;
    Polyhedron3 minkowski_diff_res;
    minkowski_sum.convert_to_polyhedron(minkowski_sum_res);
    minkowski_diff.convert_to_polyhedron(minkowski_diff_res);

    // Save the Minkowski sum and difference
    std::cout << "Saving Minkowski sum and difference to OBJ files..." << std::endl;

    if (verbose) {
        std::cout << "Minkowski sum: " << minkowski_sum.number_of_vertices() << " vertices, "
                  << minkowski_sum.number_of_halfedges() << " halfedges, " << minkowski_sum.number_of_facets()
                  << " facets" << std::endl;
    }
    std::string   minkowski_sum_filename = "./MinkowskiSum.obj";
    std::ofstream minkowski_sum_file(minkowski_sum_filename);
    if (minkowski_sum_file.fail() || !CGAL::IO::write_OBJ(minkowski_sum_file, minkowski_sum_res)) {
        std::cerr << "Failed to write file: " << minkowski_sum_filename << std::endl;
        return 1;
    }

    if (verbose) {
        std::cout << "Minkowski diff: " << minkowski_diff.number_of_vertices() << " vertices, "
                  << minkowski_diff.number_of_halfedges() << " halfedges, " << minkowski_diff.number_of_facets()
                  << " facets" << std::endl;
    }
    std::string   minkowski_diff_filename = "./MinkowskiDiff.obj";
    std::ofstream minkowski_diff_file(minkowski_diff_filename);
    if (minkowski_diff_file.fail() || !CGAL::IO::write_OBJ(minkowski_diff_file, minkowski_diff_res)) {
        std::cerr << "Failed to write file: " << minkowski_diff_filename << std::endl;
        return 1;
    }

    return 0;
}