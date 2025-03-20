package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constants.VisionConstants;

public class Camera {
  
  private final PhotonCamera m_photonCamera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final PhotonPoseEstimator m_singleTagPoseEstimator;
  private PhotonPipelineResult m_latestResult;
  private final String m_cameraName;
  private Matrix<N3, N1> m_latestStdDevs;
  private boolean m_lowStdDevs = false;

  public Camera(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, false);
  }

  public Camera(String name, Transform3d robotToCamera, boolean lowStdDevs) {
    m_cameraName = name;
    m_photonCamera = new PhotonCamera(name);
    m_lowStdDevs = lowStdDevs;
    m_singleTagPoseEstimator = new PhotonPoseEstimator(
      VisionConstants.kAprilTagLayout,
      PoseStrategy.LOWEST_AMBIGUITY,
      robotToCamera
    );

    m_poseEstimator = new PhotonPoseEstimator(
      VisionConstants.kAprilTagLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // MULTI_TAG_PNP_ON_COPROCESSOR
      robotToCamera
    );
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  // public PhotonPipelineResult getLastestCameraResult() {
  //   // boolean isNewResult = true;
    
  //   // if (m_latestResult != null) {
  //   //   double now = Timer.getFPGATimestamp(); // seconds
  //   //   isNewResult = Math.abs(now - m_latestResult.getTimestampSeconds()) > 1e-5; // ask Drew about all this stuff 
  //   // }
  //   // // System.out.println("isNewResult" + isNewResult);

  //   // if(!isNewResult)
  //   //   return m_latestResult;
    
  //   List<PhotonPipelineResult> cameraResults = m_photonCamera.getAllUnreadResults();
    
  //   if (!cameraResults.isEmpty()){
  //     PhotonPipelineResult result = cameraResults.get(cameraResults.size() - 1); // FIFO queue

  //     if (!result.hasTargets())
  //       return m_latestResult;

  //     m_latestResult = result;
  //     System.out.println("GOT RESULT " + cameraResults.size() + " " + m_cameraName);
  //   }

  //   return m_latestResult;
  // }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() { return getEstimatedGlobalPose(false);}
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean singleTag) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (PhotonPipelineResult result : m_photonCamera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      // TODO : this sucks do a steams thing when you have time

      for (PhotonTrackedTarget target : result.getTargets()) {
        for (Integer bannedTarget : VisionConstants.kBannedTagIDs) {
          if (bannedTarget == target.getFiducialId()) {
            // System.out.println("Tag being filtered");
            result = new PhotonPipelineResult();
            continue;
          }
        }
      }

      m_latestResult = result;

      if (singleTag) {
        visionEst = m_singleTagPoseEstimator.update(result);
      } else {
        visionEst = m_poseEstimator.update(result);
      }
      if (!m_lowStdDevs)
        updateEstimationStdDevs(visionEst, result.getTargets());
    }

    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    if (m_lowStdDevs)
      return VisionConstants.StdDevs.kLow;
    return m_latestStdDevs;
  }

  private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPoseOp, List<PhotonTrackedTarget> targets) {
    Matrix<N3, N1> estStdDevs = VisionConstants.StdDevs.kSingleTag;

    if (estimatedPoseOp.isEmpty()) {
      m_latestStdDevs = estStdDevs;
      return;
    }

    Pose2d estimatedPose = estimatedPoseOp.get().estimatedPose.toPose2d();

    int numTags = 0;
    double avgDist = 0;

    for (var target : targets) {
      Optional<Pose3d> tagPose = m_poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

      if (tagPose.isEmpty()) 
        continue;

      numTags++;

      Translation2d tagTranslation = tagPose.get().toPose2d().getTranslation();
      avgDist += tagTranslation.getDistance(estimatedPose.getTranslation());
    }
    
    if (numTags == 0){
      m_latestStdDevs = estStdDevs;
      return;
    }

    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.StdDevs.kMultipleTags;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    m_latestStdDevs = estStdDevs;
  }

  public AprilTag[] getSeenTags() {
    if (m_latestResult == null) {
      return new AprilTag[0];
    }

    AprilTag[] tags = new AprilTag[m_latestResult.targets.size()];

    int i = 0;
    for (PhotonTrackedTarget target : m_latestResult.targets) {
      Optional<Pose3d> tagPose = m_poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      
      tags[i] = new AprilTag(target.getFiducialId(), 
        tagPose.isEmpty() ? new Pose3d() : tagPose.get()
      );
      i++;
    }
    return tags;
  }
  
}