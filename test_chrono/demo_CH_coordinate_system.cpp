#include <ostream>
#include "chrono/physics/ChBodyAuxRef.h"
#include "chrono/physics/ChSystemNSC.h"
#include "core/ChVector3.h"
#include "utils/ChConstants.h"

using namespace chrono;

int main() {
    ChSystemNSC sys;

    // Create a body with auxiliary reference frame
    auto body = chrono_types::make_shared<ChBodyAuxRef>();

    body->SetFrameRefToAbs(ChFrame<>(ChVector3<>(0.0, 0.0, 3.0)));

    body->SetFrameCOMToRef(ChFrame<>(ChVector3<>(0.0, 0.0, 2.0)));

    body->SetAngAccLocal(ChVector3<>(0.0, 0.0, chrono::CH_PI));

    sys.AddBody(body);

    // Set the gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3<>(0.0, 0.0, 0.0));

    // Set the simulation time step
    double time_step    = 1e-3;
    double time_current = 0.0;
    double time_end     = 0.5;

    // Simulation loop
    while (time_current <= time_end) {
        // Advance simulation by one step
        sys.DoStepDynamics(time_step);

        time_current += time_step;

        // Print the kinematic information of the body
        std::cout << "Time: " << time_current << "\n"
                  << "Position: " << body->GetPos() << "\n"
                  << "Velocity: " << body->GetPosDt() << "\n"
                  << "Acceleration: " << body->GetPosDt2() << "\n"
                  << "Linear velocity: " << body->GetLinVel() << "\n"
                  << "Angular velocity: " << body->GetAngVelLocal() << std::endl;
    }

    return 0;
}