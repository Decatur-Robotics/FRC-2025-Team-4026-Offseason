package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;

    protected final Transform3d cameraToRobot;

    TargetObservation targetObservation;
    public VisionIOPhotonVision(
        Transform3d cameraToRobot,
        String cameraName
    ) {
        this.camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Set<Short> aprilTagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new java.util.ArrayList<>();
        for(var result : camera.getAllUnreadResults()){
            if(result.hasTargets()){
                targetObservation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch())
                );
            
            }
            else{
                targetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }
            if(result.multitagResult.isPresent()){
                var multitagResult = result.multitagResult.get();

                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(cameraToRobot.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                double totalTagDistance = 0.0;
                for(var target : result.getTargets()){
                    totalTagDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                aprilTagIds.addAll(multitagResult.fiducialIDsUsed);

                poseObservations.add(new PoseObservation(result.getTimestampSeconds(),
                 robotPose, multitagResult.estimatedPose.ambiguity, 
                 multitagResult.fiducialIDsUsed.size(), 
                 totalTagDistance/result.targets.size(), 
                 PoseObservationType.PHOTONVISION));

            }
            else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = aprilTagLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(cameraToRobot.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    aprilTagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robotPose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            cameraToTarget.getTranslation().getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }
        PoseObservation[] poseObservationArray = new PoseObservation[poseObservations.size()];
        for(int i = 0; i < poseObservations.size(); i++){
            poseObservationArray[i] = poseObservations.get(i);
        }

        int[] aprilTagIdArray = new int[aprilTagIds.size()];
        int i = 0;
        for(int id : aprilTagIds){
            aprilTagIdArray[i++] = id;
        }
        inputs.visionData = new VisionIOData(camera.isConnected(), targetObservation, poseObservationArray, aprilTagIdArray);

    }
}
