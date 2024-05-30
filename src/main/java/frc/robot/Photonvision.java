package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Pair;

/**
 * A subsystem for using a camera to find the robot's position. This
 * uses a software known as "PhotonVision", and the raspberry pi as a 
 * coprocessor
 * 
 * @author H!
 */
public class Photonvision {
    public Photonvision() {};

    public Optional<Pair<EstimatedRobotPose, Double>> getPhotonPose() {
        throw new UnsupportedOperationException("SubsystemPhotonvision.getPhotonPose() not implemented");
    }
}