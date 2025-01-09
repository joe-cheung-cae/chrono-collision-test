#include "assets/ChColor.h"
#include "assets/ChVisualSystem.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "collision/ChCollisionModel.h"
#include "collision/ChCollisionSystem.h"
#include "core/ChGlobal.h"
#include "core/ChTypes.h"
#include "core/ChVector3.h"
#include "physics/ChContactMaterialSMC.h"
#include "physics/ChSystemSMC.h"

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// -----------------------------------------------------------------------------

bool                    draw_coll_shapes = false;
ChCollisionSystem::Type coll_sys_type    = ChCollisionSystem::Type::BULLET;

// -----------------------------------------------------------------------------

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
        std::cout << "Contact point on A: " << contactinfo.vpA << "\n"
                  << "Contact point on B: " << contactinfo.vpB << "\n"
                  << "Contact normal: " << contactinfo.vN << "\n"
                  << "Contact distance: " << contactinfo.distance << "\n"
                  << "Effective radius of curvature: " << contactinfo.eff_radius << "\n";
        std::cout << std::endl;
    }
};

int main() {
    // Create a Chrono physical system
    ChSystemSMC sys;

    // Set the gravitational acceleration
    sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set the collision system type
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0025);
    sys.SetCollisionSystemType(coll_sys_type);

    // Set the contact call back function
    auto callback = chrono_types::make_shared<MyCallBackOnAddContack>();
    sys.GetContactContainer()->RegisterAddContactCallback(callback);

    // Create a material
    auto mat = chrono_types::make_shared<ChContactMaterialSMC>();

    // Create the ground
    auto ground = chrono_types::make_shared<ChBodyEasyMesh>(
        "/home/joe/repo/chrono-collision-test/data/obj/Ground.obj",  // Mesh obejct
        1000,                                                        // Density
        true,                                                        // Visualize
        true,                                                        // Collide
        true,                                                        // Use material properties
        mat,                                                         // Contact material
        0.005                                                        // Sphere swept
    );
    ground->GetVisualShape(0)->SetColor(ChColor(0.5f, 0.0f, 0.0f));
    ground->SetFixed(true);
    sys.AddBody(ground);

    // Create a falling box
    auto falling_box = chrono_types::make_shared<ChBodyEasyMesh>(
        "/home/joe/repo/chrono-collision-test/data/obj/FallingBoxPerpendicular.obj",  // mesh object
        1000,                                                                         // density
        true,                                                                         // visualize
        true,                                                                         // collide
        true,                                                                         // use material properties
        mat,                                                                          // contact material
        0.005                                                                         // sphere swept
    );
    falling_box->SetFixed(false);
    falling_box->SetFrameRefToAbs(ChFrame<>(ChVector3d(0.0, 0.0, 1.0)));
    sys.AddBody(falling_box);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1280, 720);
    vis->SetWindowTitle("Collisions between objects");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->AddCamera(ChVector3d(1, 1, 1));
    vis->AddLight(ChVector3d(30, 80, +30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->AddLight(ChVector3d(30, 80, -30), 80, ChColor(0.7f, 0.7f, 0.7f));
    vis->EnableShadows();

    // Time step
    double time_step    = 1e-3;
    double time_current = 0.0;
    double time_end     = 2.0;

    // Simulation loop
    while (time_current <= time_end) {
        // Render the scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Advance simulation by one step
        sys.DoStepDynamics(0.001);

        time_current += time_step;
    }

    return 0;
}