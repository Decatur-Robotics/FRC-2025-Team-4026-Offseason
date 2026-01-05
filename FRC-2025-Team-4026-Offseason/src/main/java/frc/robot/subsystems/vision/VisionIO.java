package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        VisionIOData visionData = new VisionIOData(false, false,
         new TargetObservation(new Rotation2d(), new Rotation2d()), new PoseObservation[0], new int[0]);
    }
    record VisionIOData(
        boolean leftCameraConnected,
        boolean rightCameraConnected,
        TargetObservation targetObservation,
        PoseObservation[] poseObservation,
        int[] aprilTagIds
    ){}

        record PoseObservation(
            double timestamp,
            Pose3d pose,
            double ambiguity,
            int tagCount,
            double averageTagDistance,
            PoseObservationType type) {}

            enum PoseObservationType {
                MEGATAG_1,
                MEGATAG_2,
                PHOTONVISION
            }

        record TargetObservation(Rotation2d tx, Rotation2d ty) {}

        default void updateInputs(VisionIOInputs inputs) {}
}
