package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera photonCamera;
  private AprilTagFieldLayout m_layout;
  public Field2d m_field2d = new Field2d();

  private static final String camera_name = "limelight";
  private static final Pose3d robot_to_camera = new Pose3d(0, 0, 0, new Rotation3d() );
  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */

  public PhotonVision() {
    photonCamera = new PhotonCamera(camera_name);
    try{
      m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } 
    catch(Exception error){
      throw new RuntimeException(error);
    } 
  }

  public PhotonTrackedTarget getBestTarget() {
    PhotonPipelineResult result = photonCamera.getLatestResult();

    boolean hasTarget = result.hasTargets();

    PhotonTrackedTarget target = null;

    if (hasTarget) {
        target = result.getBestTarget();
    }

    return target;
  }

  public Pose2d getLatestEstimatedRobotPose() {
    PhotonTrackedTarget target = getBestTarget();

    if (target != null) {
      Transform3d cameraToTarget = target.getBestCameraToTarget();

      Optional<Pose3d> tagPose = m_layout.getTagPose(target.getFiducialId());

      Transform3d camToRobot = new Transform3d();

      if (tagPose.isPresent()) {
          Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
          return robotPose.toPose2d();
      }
    }
    return new Pose2d();
  }

  @Override
  public void periodic() {

  }

}