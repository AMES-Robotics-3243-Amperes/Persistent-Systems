package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem for using a camera to find the robot's position. This
 * uses a software known as "PhotonVision", and the raspberry pi as a 
 * coprocessor
 * 
 * @author H!
 */
public class SubsystemPhotonvision extends SubsystemBase {
    public SubsystemPhotonvision() {};

    public Optional<EstimatedRobotPose> getPhotonPose() {
        throw new UnsupportedOperationException("SubsystemPhotonvision.getPhotonPose() not implemented");
    }
}