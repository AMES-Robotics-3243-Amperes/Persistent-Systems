package frc.robot;
/*
 * Represents a PhotonCamera with the full code pipeline built in.
 */

import java.util.ArrayList;
import java.util.List;

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
  private Transform3d robotToCamera;

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

    poseEstimator = new PhotonPoseEstimator(fieldLayout, strategy, robotToCamera);
    poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    this.robotToCamera = robotToCamera;
  }

  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  public List<Measurement> getMeasurement() {
    List<Measurement> measurements = new ArrayList<Measurement>();
    List<PhotonPipelineResult> pipelineResults = this.getAllUnreadResults();

    for (PhotonPipelineResult result : pipelineResults) {
      var poseLatestOptional = poseEstimator.update(result);

      if (poseLatestOptional.isPresent() && !result.targets.isEmpty()) {
        EstimatedRobotPose poseLatest = poseLatestOptional.get();
        if (result.getBestTarget().getPoseAmbiguity() > PhotonvisionConstants.poseEstimatorAmbiguityScaleFactor) {
          continue;
        }

        Translation2d targetPosition = poseEstimator.getFieldTags()
            .getTagPose(result.getBestTarget().getFiducialId()).get().getTranslation().toTranslation2d();
        Measurement measurement = new Measurement(poseLatest.estimatedPose.toPose2d(),
            poseLatest.timestampSeconds, result.getBestTarget().getPoseAmbiguity(), targetPosition);

        measurements.add(measurement);
      }
    }

    return measurements;
  }
}
