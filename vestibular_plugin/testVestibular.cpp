#include <OpenSim/OpenSim.h>
#include "Schneider15VestibularAfferent.h"
#include <iostream>

using namespace OpenSim;
using namespace SimTK;

int main() {
    try {
        // 1. Load the Vestibular Plugin
        LoadOpenSimLibrary("osimVestibular");

        // 2. Create the Model
        Model model;
        model.setName("VestibularStandaloneTest");
        model.setUseVisualizer(false);
        model.setGravity(Vec3(0, -9.80665, 0)); // Standard Earth gravity

        // 3. Create bodies for the serial kinematic chain
        // We use tiny masses for the dummy bodies to keep Simbody happy.
        double dm = 1e-4;
        Inertia di(1e-6, 1e-6, 1e-6, 0, 0, 0);
        OpenSim::Body* dummy1 = new OpenSim::Body("dummy1", dm, Vec3(0), di);
        OpenSim::Body* dummy2 = new OpenSim::Body("dummy2", dm, Vec3(0), di);
        OpenSim::Body* dummy3 = new OpenSim::Body("dummy3", dm, Vec3(0), di);
        OpenSim::Body* dummy4 = new OpenSim::Body("dummy4", dm, Vec3(0), di);
        
        double mass = 5.0;
        OpenSim::Body* head = new OpenSim::Body("head", mass, Vec3(0), Inertia(0.1, 0.1, 0.1, 0, 0, 0));
        head->attachGeometry(new FrameGeometry(0.2));
        
        model.addBody(dummy1);
        model.addBody(dummy2);
        model.addBody(dummy3);
        model.addBody(dummy4);
        model.addBody(head);

        // Biological normal vectors (from Aw et al 1996)
        Vec3 n_ant(-0.04, 0.65, 0.76);     n_ant = n_ant.normalize();
        Vec3 n_post(-0.64, 0.05, -0.77);   n_post = n_post.normalize();
        Vec3 n_horz(0.18, 0.94, 0.27);     n_horz = n_horz.normalize();
        Vec3 n_utr = n_horz;               
        Vec3 n_sac(0, 0, 1);               

        // Helper function to create a PinJoint aligned with a specific axis
        // PinJoint rotates about the Z axis. We need to find a rotation that maps Z to our desired normal.
        auto createJointAligned = [](const std::string& name, const PhysicalFrame& parent, const PhysicalFrame& child, Vec3 axis, bool isPin) -> Joint* {
            // Find rotation that aligns Z (0,0,1) with axis
            UnitVec3 z(0,0,1);
            UnitVec3 v(axis);
            Rotation rot;
            rot.setRotationFromTwoAxes(z, ZAxis, v, XAxis); // Wait, setRotationFromOneAxis is better
            // Actually, SimTK::Rotation(unitVec1, axis1, unitVec2, axis2)
            // A simpler way: Rotation().setRotationFromAngleAboutAxis(...) isn't quite right.
            // SimTK::Rotation has a constructor that aligns a specific axis.
            // Let's just construct a Rotation that maps Z to 'axis'.
            // cross product of Z and axis gives the axis of rotation, and dot product gives angle.
            double angle = std::acos(dot(Vec3(0,0,1), axis));
            Vec3 cross = Vec3(0,0,1) % axis;
            if (cross.norm() < 1e-6) cross = Vec3(1,0,0);
            Rotation orientation(angle, cross);
            
            Vec3 location(0);
            
            if (isPin) {
                PinJoint* j = new PinJoint(name, parent, location, orientation.convertRotationToBodyFixedXYZ(), child, location, orientation.convertRotationToBodyFixedXYZ());
                return j;
            } else {
                SliderJoint* j = new SliderJoint(name, parent, location, orientation.convertRotationToBodyFixedXYZ(), child, location, orientation.convertRotationToBodyFixedXYZ());
                // SliderJoint translates along X axis in OpenSim!
                // Wait! PinJoint rotates about Z, SliderJoint translates about X.
                return j;
            }
        };

        // Wait, SliderJoint translates along X axis.
        // PinJoint rotates about Z axis.
        auto createPinZ = [](const std::string& name, const PhysicalFrame& p, const PhysicalFrame& c, Vec3 axis) {
            Vec3 rotAxis = (Vec3(0,0,1) % axis).normalize();
            double angle = std::acos(dot(Vec3(0,0,1), axis));
            if (std::isnan(angle) || (Vec3(0,0,1) % axis).norm() < 1e-5) { rotAxis = Vec3(1,0,0); angle = (axis[2] < 0) ? SimTK::Pi : 0; }
            Vec3 euler = Rotation(angle, rotAxis).convertRotationToBodyFixedXYZ();
            PinJoint* j = new PinJoint(name, p, Vec3(0), euler, c, Vec3(0), euler);
            j->updCoordinate().setName(name);
            return j;
        };
        auto createSliderX = [](const std::string& name, const PhysicalFrame& p, const PhysicalFrame& c, Vec3 axis) {
            Vec3 rotAxis = (Vec3(1,0,0) % axis).normalize();
            double angle = std::acos(dot(Vec3(1,0,0), axis));
            if (std::isnan(angle) || (Vec3(1,0,0) % axis).norm() < 1e-5) { rotAxis = Vec3(0,1,0); angle = (axis[0] < 0) ? SimTK::Pi : 0; }
            Vec3 euler = Rotation(angle, rotAxis).convertRotationToBodyFixedXYZ();
            SliderJoint* j = new SliderJoint(name, p, Vec3(0), euler, c, Vec3(0), euler);
            j->updCoordinate().setName(name);
            return j;
        };

        model.addJoint(createPinZ("rot_anterior", model.getGround(), *dummy1, n_ant));
        model.addJoint(createPinZ("rot_posterior", *dummy1, *dummy2, n_post));
        model.addJoint(createPinZ("rot_horizontal", *dummy2, *dummy3, n_horz));
        model.addJoint(createSliderX("trans_utricle", *dummy3, *dummy4, n_utr));
        model.addJoint(createSliderX("trans_saccule", *dummy4, *head, n_sac));

        // 5. Add the Vestibular Sensor
        Schneider15VestibularAfferent* vest = new Schneider15VestibularAfferent();
        vest->setName("vestibular_sensor");
        vest->updSocket("head_frame").connect(*head);
        
        // Adjust deadbands slightly for the test profile to ensure clear outputs
        vest->set_canal_threshold(0.5);   // 0.5 deg/s threshold
        vest->set_otolith_threshold(0.01); // 0.01 G threshold
        model.addModelComponent(vest);

        // Initialize system so we can access and prescribe the coordinates
        model.initSystem();

        // 6. Prescribe specific motion profiles (StepFunction provides smooth position steps, creating bell-shaped velocity bumps)
        auto setPrescribed = [&](const std::string& name, double t0, double t1, double val0, double val1) {
            Coordinate& coord = model.updCoordinateSet().get(name);
            coord.setDefaultIsPrescribed(true);
            StepFunction* func = new StepFunction(t0, t1, val0, val1);
            coord.setPrescribedFunction(*func);
        };

        // Profile: Each coordinate activates for 2 seconds, sequentially.
        setPrescribed("rot_anterior",   1.0,  3.0, 0.0, 1.0);  // 1 rad (~57 deg) rotation
        setPrescribed("rot_posterior",  5.0,  7.0, 0.0, 1.0);
        setPrescribed("rot_horizontal", 9.0, 11.0, 0.0, 1.0);
        setPrescribed("trans_utricle", 13.0, 15.0, 0.0, 1.0);  // 1 meter translation
        setPrescribed("trans_saccule", 17.0, 19.0, 0.0, 1.0);

        // 7. Setup the Simulator
        State& state = model.initSystem();
        Manager manager(model);
        manager.setIntegratorAccuracy(1.0e-5);
        manager.setIntegratorMinimumStepSize(1.0e-8);
        manager.initialize(state);

        std::cout << "Running Vestibular Standalone Test (20 seconds)..." << std::endl;
        
        // 8. Integrate forward in time
        manager.integrate(20.0);
        
        // 9. Save Results
        auto statesTable = manager.getStatesTable();
        STOFileAdapter::write(statesTable, "vestibular_standalone_results.sto");
        model.print("VestibularStandaloneTest.osim");
        
        std::cout << "Success!" << std::endl;
        std::cout << "Saved model to: VestibularStandaloneTest.osim" << std::endl;
        std::cout << "Saved states to: vestibular_standalone_results.sto" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
