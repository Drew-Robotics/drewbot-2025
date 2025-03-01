package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Arrays;
import java.util.Comparator;
import java.util.stream.Collectors;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;

import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SubsystemAbstract;

public class VisionSubsystem extends SubsystemAbstract {
  // private final Camera m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final List<Camera> m_cameras;

  // private AprilTag[] m_fieldTags = VisionConstants.AprilTags.kTags.toArray(AprilTag[]::new);

  private final List<String> m_cameraNames = List.of(
    VisionConstants.CameraNames.kFrontLeft, 
    VisionConstants.CameraNames.kFrontRight,
    VisionConstants.CameraNames.kBackLeft,
    VisionConstants.CameraNames.kBackRight
  );

  Camera m_frontLeft, m_frontRight, m_backLeft, m_backRight;

  // private StructArrayTopicSup<AprilTag> m_seenTagsTopicSup;

  private static VisionSubsystem m_instance;
  public static VisionSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new VisionSubsystem();
    return m_instance;
  }

  private VisionSubsystem() {
    super();

    m_frontLeft = new Camera(VisionConstants.CameraNames.kFrontLeft, VisionConstants.CameraTransforms.kFrontLeft);  
    m_frontRight = new Camera(VisionConstants.CameraNames.kFrontRight, VisionConstants.CameraTransforms.kFrontRight);
    m_backLeft = new Camera(VisionConstants.CameraNames.kBackLeft, VisionConstants.CameraTransforms.kBackLeft);
    m_backRight = new Camera(VisionConstants.CameraNames.kBackRight, VisionConstants.CameraTransforms.kBackRight);

    m_cameras = List.of(m_frontLeft, m_frontRight, m_backLeft, m_backRight);
    // m_cameras = List.of();

  }

  /* ----- OVERRIDES ----- */

  @Override
  public void periodic() {
    super.periodic();
  }

  @Override
  public void dashboardInit() {}
  @Override
  public void dashboardPeriodic() {}

  @Override
  public void publishInit() {
    // addTopicSup(
    //   new StructArrayTopicSup<AprilTag>(
    //     m_table.getStructArrayTopic("Field Tags", new AprilTagStruct()).publish(),
    //     m_fieldTags
    //   )
    // );

    // m_seenTagsTopicSup = (StructArrayTopicSup<AprilTag>) addTopicSup(
    //   new StructArrayTopicSup<AprilTag>(
    //     m_table.getStructArrayTopic("Seen Tags", new AprilTagStruct()).publish(),
    //     this::getSeenTags
    //   )
    // );
  }

  @Override
  public void publishPeriodic() {}

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

  public Optional<AprilTag> getClosestTag(List<Integer> acceptedTagsIDs) {
    List<AprilTag> tags = getSeenTags().stream()
      .filter(tag -> acceptedTagsIDs.contains(tag.ID)).toList();

    if (tags.size() == 0){
      return Optional.empty();
    }

    Translation3d robotTranslation3d = subsystems.drive.getPose3d().getTranslation();
    tags.sort(Comparator.comparingDouble(
      tag -> tag.pose.getTranslation().getDistance(robotTranslation3d)
    ));

    return Optional.of(tags.get(0));
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
        
        poseStdDevs.add(Optional.ofNullable(camera.getEstimationStdDevs()));
      }
      else
        poseStdDevs.add(Optional.empty());
    }
    return poseStdDevs;
  }

}
