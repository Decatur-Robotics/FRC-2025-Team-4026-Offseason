package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.LinkedList;
import java.util.List;
import java.util.logging.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private VisionIOInputsAutoLogged[] inputs;
    private double maxAmbiguity;
    private double maxZError;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;
        maxAmbiguity =5;
        maxZError = 100;
        inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

    }

    public Rotation2d getTarget(int cameraIndex){
        return inputs[cameraIndex].visionData.targetObservation().tx();
    }

    public void periodic() {

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            org.littletonrobotics.junction.Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        List<Pose3d> allAprilTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        for(int cameraIndex = 0; cameraIndex < io.length; cameraIndex++){
            List<Pose3d> aprilTagPoses = new LinkedList<>();
        List<Pose3d> robotPoses = new LinkedList<>();
        List<Pose3d> robotPosesAccepted = new LinkedList<>();
        List<Pose3d> robotPosesRejected = new LinkedList<>();
        for(int tagId : inputs[cameraIndex].visionData.aprilTagIds()){
            var tagPose = aprilTagLayout.getTagPose(tagId);
            if(tagPose.isPresent()){
                aprilTagPoses.add(tagPose.get());
            }
        }

        for(var observation : inputs[cameraIndex].visionData.poseObservation()){
            boolean rejectPose = observation.tagCount() == 0 
            || (observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) 
            || Math.abs(observation.pose().getZ()) > maxZError 
            ||observation.pose().getX() < 0.0
            ||observation.pose().getX() > aprilTagLayout.getFieldLength()
            ||observation.pose().getY() < 0.0
            ||observation.pose().getY() > aprilTagLayout.getFieldWidth();
            
        }
        }
        

    }

        @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
