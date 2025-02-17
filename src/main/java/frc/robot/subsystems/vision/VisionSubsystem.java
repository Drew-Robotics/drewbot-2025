package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.LoggerI;
import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.subsystems.topicSup.StructArrayTopicSup;

public class VisionSubsystem extends SubsystemAbstract {
  // private final Camera m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final List<Camera> m_cameras;

  private AprilTag[] m_fieldTags = VisionConstants.AprilTags.kTags.toArray(AprilTag[]::new);

  private final List<String> m_cameraNames = List.of(
    VisionConstants.CameraNames.kFrontLeft, 
    VisionConstants.CameraNames.kFrontRight,
    VisionConstants.CameraNames.kBackLeft,
    VisionConstants.CameraNames.kBackRight
  );

  private StructArrayTopicSup<AprilTag> m_seenTagsTopicSup;

  private static class VisionLogger implements LoggerI {
    private VisionSubsystem m_vision = subsystems.vision;

    private StructArrayPublisher<AprilTag> m_fieldTags = m_vision.m_table.getStructArrayTopic("Field Tags", new AprilTagStruct()).publish();
    private StructArrayPublisher<AprilTag> m_seenTags = m_vision.m_table.getStructArrayTopic("Seen Tags", new AprilTagStruct()).publish();

    public void publishPeriodic() {
        m_fieldTags.accept(subsystems.vision.m_fieldTags);
        m_seenTags.accept(subsystems.vision.getSeenTags().toArray(AprilTag[]::new));
    }
}

  private static VisionSubsystem m_instance;
  public static VisionSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new VisionSubsystem();
    return m_instance;
  }

  private VisionSubsystem() {
    super(new VisionLogger());

    // m_frontLeft = new Camera(VisionConstants.CameraNames.kFrontLeft, VisionConstants.CameraTransforms.kFrontLeft);  
    // m_frontRight = new Camera(VisionConstants.CameraNames.kFrontRight, VisionConstants.CameraTransforms.kFrontRight);
    // m_backLeft = new Camera(VisionConstants.CameraNames.kBackLeft, VisionConstants.CameraTransforms.kBackLeft);
    // m_backRight = new Camera(VisionConstants.CameraNames.kBackRight, VisionConstants.CameraTransforms.kBackRight);

    // m_cameras = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
    m_cameras = List.of();

  }

  /* ----- OVERRIDES ----- */

  @Override
  public void periodic() {
    super.periodic();
  }

  // @Override
  // public void publishInit() {
  //   addTopicSup(
  //     new StructArrayTopicSup<AprilTag>(
  //       m_table.getStructArrayTopic("Field Tags", new AprilTagStruct()).publish(),
  //       m_fieldTags
  //     )
  //   );

  //   m_seenTagsTopicSup = (StructArrayTopicSup<AprilTag>) addTopicSup(
  //     new StructArrayTopicSup<AprilTag>(
  //       m_table.getStructArrayTopic("Seen Tags", new AprilTagStruct()).publish(),
  //       this::getSeenTags
  //     )
  //   );
  // }

  /* ----- VISION ------ */

  public List<AprilTag> getSeenTags(){
    return new ArrayList<AprilTag>(m_cameras.stream()
      .flatMap((Camera cam) -> Arrays.stream(cam.getSeenTags()))
      .collect(Collectors.toMap(
        tag -> tag.ID,
        tag -> tag,
        (e, r) -> e // discards any repeating tags
      )).values());
  }

  public List<Optional<EstimatedRobotPose>> getCameraEstimatedPoses() {
    ArrayList<Optional<EstimatedRobotPose>> retVal = new ArrayList<Optional<EstimatedRobotPose>>();
    for(Camera camera : m_cameras) {
      retVal.add(camera.getEstimatedGlobalPose());
    }
    return retVal;
  }

  public List<Optional<Matrix<N3, N1>>> getPoseStdDevs(List<Optional<EstimatedRobotPose>> poses) {
    ArrayList<Optional<Matrix<N3, N1>>> poseStdDevs = new ArrayList<Optional<Matrix<N3, N1>>>();

    for (int cameraIndex = 0; cameraIndex < m_cameras.size(); cameraIndex++) {
      Camera camera = m_cameras.get(cameraIndex);

      Optional<EstimatedRobotPose> poseOptional = poses.get(cameraIndex);

      SmartDashboard.putBoolean(m_cameraNames.get(cameraIndex) + " Present", poseOptional.isPresent());
      
      if(poseOptional.isPresent()){
        Pose2d pos = poseOptional.get().estimatedPose.toPose2d();
        SmartDashboard.putString(m_cameraNames.get(cameraIndex) + " Pose", pos.toString());
        
        poseStdDevs.add(Optional.ofNullable(camera.getEstimationStdDevs(
          poseOptional.get().estimatedPose.toPose2d()
        )));
      }
      else
        poseStdDevs.add(Optional.empty());
    }
    return poseStdDevs;
  }

}