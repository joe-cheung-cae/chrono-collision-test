#include "assets/ChVisualSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "core/ChTypes.h"
#include "core/ChVector3.h"
#include "geometry/ChTriangleMeshConnected.h"
#include "physics/ChBody.h"
#include "physics/ChContactMaterialNSC.h"
#include <iostream>
#include <iomanip>

using namespace chrono;
using namespace chrono::irrlicht;

class MyCallBackOnAddContack : public ChContactContainer::AddContactCallback {
  public:
    /// Callback used to process contact points being added to the container.
    /// A derived user-provided callback class must implement this. The provided
    /// composite material should be downcast to the appropriate type.
    virtual void OnAddContact(const ChCollisionInfo&            contactinfo,  ///< information about the collision pair
                              ChContactMaterialComposite* const material      ///< composite material can be modified
                              ) override {
        // Print contact information
        std::cout << "\n";
        std::cout << "Time: " << std::setprecision(8) << contactinfo.modelA->GetPhysicsItem()->GetChTime() << "\n"
                  << "Contact point on A: " << contactinfo.vpA << "\n"
                  << "Contact point on B: " << contactinfo.vpB << "\n"
                  << "Contact normal: " << contactinfo.vN << "\n"
                  << "Contact distance: " << contactinfo.distance << "\n"
                  << "Name A:" << contactinfo.modelA->GetPhysicsItem()->GetName() << "\n"
                  << "Name B:" << contactinfo.modelB->GetPhysicsItem()->GetName() << "\n";
        std::cout << std::endl;
    }
};

int main() {
    // Create a Chrono physical system
    ChSystemNSC sys;

    // Set the gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set the contact call back function
    auto my_callback = chrono_types::make_shared<MyCallBackOnAddContack>();
    sys.GetContactContainer()->RegisterAddContactCallback(my_callback);

    // Create a contact material
    auto material = chrono_types::make_shared<ChContactMaterialNSC>();
    material->SetFriction(0.5f);
    material->SetRestitution(0.1f);

    // Create a ground
    auto ground_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile("../data/obj/Ground.obj");
    auto ground      = chrono_types::make_shared<ChBodyEasyMesh>(ground_mesh, 1000, true, true, true, material);
    ground->SetName("Ground");
    ground->SetFixed(true);
    sys.AddBody(ground);

    // Create a falling box
    auto falling_box_mesh = ChTriangleMeshConnected::CreateFromWavefrontFile("../data/obj/FallingBoxPerpendicular.obj");
    auto falling_box = chrono_types::make_shared<ChBodyEasyMesh>(falling_box_mesh, 1000, true, true, true, material);
    falling_box->SetName("FallingBox");
    falling_box->SetFrameRefToAbs(ChFrame<>(ChVector3d(0.35, 0.35, 0.75)));
    sys.AddBody(falling_box);

    // Create the visulization engine
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("Chrono collision profile");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(2, 2, 1), ChVector3d(0.0, 0.0, 0.0));
    vis->AddLight(ChVector3d(30, 80, +30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector3d(30, 80, -30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->EnableShadows();

    // Set up the simulation loop
    double time_current = 0.0;
    double time_step    = 0.001;
    double time_end     = 1.0;

    // Run the simulation loop
    while (time_current <= time_end) {
        // Render the scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance simulation by one step
        sys.DoStepDynamics(time_step);

        // Increment time
        time_current += time_step;
    }

    return 0;
}