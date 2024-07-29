package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import frc.robot.Constants.PhotonvisionConstants;

/**
 * A subsystem for using a camera to find the robot's position. This
 * uses a software known as "PhotonVision", and the raspberry pi as a
 * coprocessor
 * 
 * @author H!
 */
public class Photonvision {
  protected PhotonCamera camera;
  protected AprilTagFieldLayout fieldLayout;
  protected PhotonPoseEstimator poseEstimator;

  /** Creates a new SubsystemPhotonVision. */
  public Photonvision() throws IOException {
    camera = new PhotonCamera(PhotonvisionConstants.cameraName);
    fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    //fieldLayout = new AprilTagFieldLayout(tags, Units.inchesToMeters(480), Units.inchesToMeters(288));

    poseEstimator = new PhotonPoseEstimator(fieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      PhotonvisionConstants.robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<Pair<EstimatedRobotPose, Double>> getPhotonPose() {
    var poseLatestOptional = poseEstimator.update();
    PhotonPipelineResult pipelineResult = camera.getLatestResult();
    
    if (poseLatestOptional.isPresent() && pipelineResult.targets.size() != 0) {
      EstimatedRobotPose poseLatest = poseLatestOptional.get();
      return Optional.of(new Pair<EstimatedRobotPose, Double>(poseLatest, pipelineResult.getBestTarget().getPoseAmbiguity()));
    }

    return Optional.empty();
  }
}