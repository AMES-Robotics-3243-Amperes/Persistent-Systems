package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  protected PhotonCamera camera2;
  protected AprilTagFieldLayout fieldLayout;
  protected PhotonPoseEstimator poseEstimator;
  protected PhotonPoseEstimator poseEstimator2;

  /** Creates a new SubsystemPhotonVision. */
  public Photonvision() throws IOException {
    camera = new PhotonCamera("Global_Shutter_Camera");
    camera2 = new PhotonCamera("CameraB");
    AprilTag tag = new AprilTag(1, new Pose3d(new Translation3d(1, 0, 0), new Rotation3d(0, 0, Math.PI)));
    ArrayList<AprilTag> tags = new ArrayList<>();
    tags.add(tag);
    fieldLayout = new AprilTagFieldLayout(tags, 20, 20);

    poseEstimator = new PhotonPoseEstimator(fieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera,
      PhotonvisionConstants.robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);

    poseEstimator2 = new PhotonPoseEstimator(fieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      camera2,
      PhotonvisionConstants.robotToCamera);
    poseEstimator2.setMultiTagFallbackStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
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

  public Optional<Pair<EstimatedRobotPose, Double>> getPhotonPose2() {
    var poseLatestOptional2 = poseEstimator2.update();
    PhotonPipelineResult pipelineResult2 = camera2.getLatestResult();

    if (poseLatestOptional2.isPresent() && pipelineResult2.targets.size() != 0) {
      EstimatedRobotPose poseLatest2 = poseLatestOptional2.get();
      return Optional.of(new Pair<EstimatedRobotPose, Double>(poseLatest2, pipelineResult2.getBestTarget().getPoseAmbiguity()));
    }

    return Optional.empty();
  }
}