package frc.robot;
/*
 * Represents a PhotonCamera with the full code pipeline built in.
 */

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.PhotonvisionConstants;

public class PhotonUnit extends PhotonCamera {
  private PhotonPoseEstimator poseEstimator;

  public class Measurement {
    public Pose2d pose;
    public double timestampSeconds;
    public Vector<N3> ambiguity;
    public Translation2d targetPosition;

    public Measurement(Pose2d pose, double timestampSeconds, double ambiguity, Translation2d targetPosition) {
      this.pose = pose;
      this.timestampSeconds = timestampSeconds;
      this.ambiguity = VecBuilder.fill(ambiguity, ambiguity, ambiguity);
      this.targetPosition = targetPosition;
    }
  }

  public PhotonUnit(String cameraName, PoseStrategy strategy, Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout) {
    super(cameraName);

    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, this, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public Optional<Measurement> getMeasurement() {
    var poseLatestOptional = poseEstimator.update();
    PhotonPipelineResult pipelineResult = this.getLatestResult();

    if (poseLatestOptional.isPresent() && pipelineResult.targets.size() != 0) {
      EstimatedRobotPose poseLatest = poseLatestOptional.get();
      if (pipelineResult.getBestTarget().getPoseAmbiguity() > PhotonvisionConstants.poseEstimatorAmbiguityScaleFactor) {
        return Optional.empty();
      }

      Translation2d targetPosition = poseEstimator.getFieldTags()
          .getTagPose(pipelineResult.getBestTarget().getFiducialId()).get().getTranslation().toTranslation2d();
      Measurement measurement = new Measurement(poseLatest.estimatedPose.toPose2d(),
          poseLatest.timestampSeconds, pipelineResult.getBestTarget().getPoseAmbiguity(), targetPosition);

      return Optional.of(measurement);
    }

    return Optional.empty();
  }
}
