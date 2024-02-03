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

import frc.robot.Constants.VisionConstants;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera photonCamera;
  private AprilTagFieldLayout m_layout;
  public Field2d m_field2d = new Field2d();

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */

  public PhotonVision() {
    photonCamera = new PhotonCamera(VisionConstants.CAMERA_NAME);
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
    
    // // Update pose estimator with the best visible target
    // var pipelineResult = photonCamera.getLatestResult();
    // var resultTimestamp = pipelineResult.getTimestampSeconds();
    // if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
    //   previousPipelineTimestamp = resultTimestamp;
    //   var target = pipelineResult.getBestTarget();
    //   var fiducialId = target.getFiducialId();
    //   // Get the tag pose from field layout - consider that the layout will be null if it failed to load
    //   Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
    //   if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
    //     var targetPose = tagPose.get();
    //     Transform3d camToTarget = target.getBestCameraToTarget();
    //     Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

    //     var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
    //     poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
    //   }
    // }
    // // Update pose estimator with drivetrain sensors
    // poseEstimator.update(
    //   swerve.getGyroscopeRotation(),
    //   swerve.getModulePositions());

    // field2d.setRobotPose(getCurrentPose());
  }

}