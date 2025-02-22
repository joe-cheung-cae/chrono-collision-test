#include "chrono/core/ChMatrix33.h"
#include "core/ChMatrix.h"
#include "core/ChVector3.h"

using namespace chrono;

int main() {
    ChMatrix33<> test(ChVector3<>(0.000172933, -1.13132e-12, -2.55042e-12),
                      ChVector3<>(-1.13132e-12, 9.6639e-05, -7.6294e-05),
                      ChVector3<>(-2.55042e-12, -7.6294e-05, 9.6639e-05));

    ChVectorN<double, 3> eigen_values;
    ChMatrix33<>         eigen_vectors;

    test.SelfAdjointEigenSolve(eigen_vectors, eigen_values);

    // Print the eigen values
    std::cout << "Eigenvalues: \n" << eigen_values << std::endl;

    // Print the eigen vectors
    std::cout << "Eigenvectors: \n" << eigen_vectors << std::endl;

    return 0;
}