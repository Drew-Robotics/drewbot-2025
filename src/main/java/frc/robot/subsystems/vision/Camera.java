package frc.robot.subsystems.vision;

import java.util.ArrayList;
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
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.VisionConstants;

public class Camera {
  public enum SpecialCameras {
    LLFront,
    LLBack,
    None
  }

  public final String kName;

  private final PhotonCamera m_photonCamera;
  private final PhotonPoseEstimator m_poseEstimator;
  private final PhotonPoseEstimator m_singleTagPoseEstimator;
  private PhotonPipelineResult m_latestResult;
  private Matrix<N3, N1> m_latestStdDevs;
  private SpecialCameras m_specialCamera;

  private List<Integer> m_bannedTagIDs = new ArrayList<>(VisionConstants.kBannedTagIDs);

  public Camera(String name, Transform3d robotToCamera) {
    this(name, robotToCamera, SpecialCameras.None);
  }

  public Camera(String name, Transform3d robotToCamera, SpecialCameras specialCamera) {
    kName = name;
    m_photonCamera = new PhotonCamera(name);
    m_specialCamera = specialCamera;
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
    
    if (subsystems.drive.isRedAlliance()) {
      m_bannedTagIDs.addAll(VisionConstants.kBlueReefTagIDs);
    }

    if (subsystems.drive.isBlueAlliance()) {
      m_bannedTagIDs.addAll(VisionConstants.kRedReefTagIDs);
    }

  }

  public SpecialCameras getSpecialCamera() {
    return m_specialCamera;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() { return getEstimatedGlobalPose(false);}
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean singleTag) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (PhotonPipelineResult result : m_photonCamera.getAllUnreadResults()) {
      if (!result.hasTargets()) continue;

      // TODO : this sucks do a steams thing when you have time

      for (PhotonTrackedTarget target : result.getTargets()) {
        for (Integer bannedTarget : m_bannedTagIDs) {
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

      switch (m_specialCamera) {
        case LLBack:
          m_latestStdDevs = updateEstimationStdDevs(
            VisionConstants.StdDevs.kLLBackSingle,
            VisionConstants.StdDevs.kLLFrontMulti,
            visionEst, result.getTargets()
          );
          break;

        case LLFront:
          m_latestStdDevs = updateEstimationStdDevs(
            VisionConstants.StdDevs.kLLFrontSingle,
            VisionConstants.StdDevs.kLLFrontMulti,
            visionEst, result.getTargets()
          );
          break;

        default:
          m_latestStdDevs = updateEstimationStdDevs(
            VisionConstants.StdDevs.kSingle,
            VisionConstants.StdDevs.kMulti,
            visionEst, result.getTargets()
          );
          break;
      }
        
    }

    return visionEst;
  }

  public Matrix<N3, N1> getEstimationStdDevs() {
    return m_latestStdDevs;
  }

  private Matrix<N3,N1> updateEstimationStdDevs(
      Matrix<N3,N1> singleTagStdDevs,
      Matrix<N3,N1> multiTagStdDevs,
      Optional<EstimatedRobotPose> estimatedPoseOp, 
      List<PhotonTrackedTarget> targets
    )
  {
    Matrix<N3, N1> estStdDevs = singleTagStdDevs;

    if (estimatedPoseOp.isEmpty()) {
      return estStdDevs;
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
      return estStdDevs;
    }

    avgDist /= numTags;

    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = multiTagStdDevs;

    // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 4)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
      estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 100));

    return estStdDevs;
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
 
  // private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPoseOp, List<PhotonTrackedTarget> targets) {
  //   Matrix<N3, N1> estStdDevs = VisionConstants.StdDevs.kSingleTag;

  //   if (estimatedPoseOp.isEmpty()) {
  //     m_latestStdDevs = estStdDevs;
  //     return;
  //   }

  //   Pose2d estimatedPose = estimatedPoseOp.get().estimatedPose.toPose2d();

  //   int numTags = 0;
  //   double avgDist = 0;

  //   for (var target : targets) {
  //     Optional<Pose3d> tagPose = m_poseEstimator.getFieldTags().getTagPose(target.getFiducialId());

  //     if (tagPose.isEmpty()) 
  //       continue;

  //     numTags++;

  //     Translation2d tagTranslation = tagPose.get().toPose2d().getTranslation();
  //     avgDist += tagTranslation.getDistance(estimatedPose.getTranslation());
  //   }
    
  //   if (numTags == 0){
  //     m_latestStdDevs = estStdDevs;
  //     return;
  //   }

  //   avgDist /= numTags;

  //   // Decrease std devs if multiple targets are visible
  //   if (numTags > 1)
  //     estStdDevs = VisionConstants.StdDevs.kMultipleTags;

  //   // Increase std devs based on (average) distance
  //   if (numTags == 1 && avgDist > 4)
  //     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  //   else
  //     estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

  //   m_latestStdDevs = estStdDevs;
  // }
}