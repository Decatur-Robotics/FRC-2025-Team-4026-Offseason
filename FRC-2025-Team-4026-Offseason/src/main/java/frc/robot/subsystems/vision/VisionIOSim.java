package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOSim extends VisionIOPhotonVision{
    private static VisionSystemSim visionSim;

    private Supplier<Pose2d> poseSupplier;
    private PhotonCameraSim cameraSim;
    private PhotonCameraSim rightCameraSim;

    public VisionIOSim(
        Transform3d cameraToRobot,
        String cameraName,
        Supplier<Pose2d> poseSupplier
    ) {
        super(cameraToRobot,
        cameraName);
        this.poseSupplier = poseSupplier;

       if (visionSim == null) {
            visionSim = new VisionSystemSim("Main");
            visionSim.addAprilTags(aprilTagLayout);

        }

        var cameraProperties = new SimCameraProperties();

        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, cameraToRobot);

       }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}


