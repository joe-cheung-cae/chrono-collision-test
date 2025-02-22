#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "core/ChTypes.h"
#include "core/ChVector3.h"
#include "physics/ChContactMaterialNSC.h"

using namespace chrono;

int main() {
    // Create a Chrono physical system
    ChSystemNSC sys;

    // Set the gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Create a contact material
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();

    // Create a ground
    auto ground = chrono_types::make_shared<ChBodyEasyMesh>("../data/stl/FallingBoxPerpendicular.stl", 1000, true, true,
                                                            true, material);
    ground->SetFixed(true);
    sys.AddBody(ground);

    // Create a falling box
    auto falling_box = chrono_types::make_shared<ChBodyEasyMesh>("../data/stl/FallingBoxPerpendicular.stl", 1000, true,
                                                                 true, true, material);
    sys.AddBody(falling_box);

    return 0;
}