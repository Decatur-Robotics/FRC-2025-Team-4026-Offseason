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
    private PhotonCameraSim leftCameraSim;
    private PhotonCameraSim rightCameraSim;

    public VisionIOSim(
        Transform3d leftCameraToRobot,
        Transform3d rightCameraToRobot,
        Supplier<Pose2d> poseSupplier
    ) {
        super(leftCameraToRobot, rightCameraToRobot);
        this.poseSupplier = poseSupplier;

       if (visionSim == null) {
            visionSim = new VisionSystemSim("Main");
            visionSim.addAprilTags(aprilTagLayout);

        }

        var leftCameraProperties = new SimCameraProperties();
        var rightCameraProperties = new SimCameraProperties();

        leftCameraSim = new PhotonCameraSim(leftCamera, leftCameraProperties);
        rightCameraSim = new PhotonCameraSim(rightCamera, rightCameraProperties);
        visionSim.addCamera(leftCameraSim, leftCameraToRobot);
        visionSim.addCamera(rightCameraSim, rightCameraToRobot);

       }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}


