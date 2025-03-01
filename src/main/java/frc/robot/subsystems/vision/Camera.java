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
  private PhotonPipelineResult m_latestResult;
  private final String m_cameraName;

  public Camera(String name, Transform3d robotToCamera) {
    m_cameraName = name;
    m_photonCamera = new PhotonCamera(name);
    m_poseEstimator = new PhotonPoseEstimator(
      VisionConstants.kAprilTagLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      robotToCamera
    );
  }

  public PhotonPipelineResult getLastestCameraResult() {
    // boolean isNewResult = true;
    
    // if (m_latestResult != null) {
    //   double now = Timer.getFPGATimestamp(); // seconds
    //   isNewResult = Math.abs(now - m_latestResult.getTimestampSeconds()) > 1e-5; // ask Drew about all this stuff 
    // }
    // // System.out.println("isNewResult" + isNewResult);

    // if(!isNewResult)
    //   return m_latestResult;
    
    List<PhotonPipelineResult> cameraResults = m_photonCamera.getAllUnreadResults();
    
    if (cameraResults.size() > 0){
      PhotonPipelineResult result = cameraResults.get(0);

      // if (!result.hasTargets())
      //   return m_latestResult;

      m_latestResult = result;
      System.out.println("GOT RESULT " + cameraResults.size() + " " + m_cameraName);
    }

    return m_latestResult;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    if (getLastestCameraResult() == null)
      return Optional.empty();


    Optional<EstimatedRobotPose> visionEst = m_poseEstimator.update(getLastestCameraResult());
    
    if (m_cameraName == "frontLeft") {
      System.out.print("frontLeft polled, num targets : " + getLastestCameraResult().targets.size() + " ");
      System.out.println(visionEst.isPresent() + " " + getLastestCameraResult().getTimestampSeconds());
    }
    
      
    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    if (getLastestCameraResult() == null)
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);

    Matrix<N3, N1> estStdDevs = VisionConstants.StdDevs.kSingleTag;
    List<PhotonTrackedTarget> targets = getLastestCameraResult().getTargets();

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
    
    if (numTags == 0)
      return estStdDevs;

    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.StdDevs.kMultipleTags;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  public AprilTag[] getSeenTags() {
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