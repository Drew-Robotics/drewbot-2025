package frc.robot.subsystems.drive;


import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindThenFollowPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;
import frc.robot.constants.DriveAutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.RobotContainer.subsystems;
import frc.robot.subsystems.SubsystemAbstract;
import frc.robot.subsystems.drive.ReefSide.ReefBranch;

public class DriveSubsystem extends SubsystemAbstract {
  private final AHRS m_gyro;
  private boolean m_isFieldOriented = DriveConstants.kFieldOriented;

  private final SwerveModule m_frontLeft, m_frontRight, m_backLeft, m_backRight;
  private final SwerveModule[] m_modules;

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final SlewRateLimiter m_magnitudeLimiter, m_directionalLimiter, m_rotationLimiter;

  private LinearVelocity m_xVelocity = MetersPerSecond.zero();
  private LinearVelocity m_yVelocity = MetersPerSecond.zero();
  private AngularVelocity m_rotationalVelocity = RadiansPerSecond.zero();

  private PIDController m_rotationController = new PIDController(
    DriveAutoConstants.TurningPID.kP,
    DriveAutoConstants.TurningPID.kI,
    DriveAutoConstants.TurningPID.kD
  );

  private static DriveSubsystem m_instance;
  public static DriveSubsystem getInstance() {
    if (m_instance == null)
      m_instance = new DriveSubsystem();
    return m_instance;
  }

  protected DriveSubsystem() {
    super();
    m_gyro = new AHRS(NavXComType.kMXP_SPI);

    m_frontLeft = new SwerveModule(
      "Front Left",
      DriveConstants.DriveCANIDs.kFrontLeftDriving,
      DriveConstants.DriveCANIDs.kFrontLeftTurning,
      DriveConstants.AngularOffsets.kFrontLeft,
      m_table.getSubTable("SwerveModules"),
      DriveConstants.DrivingInverted.kFrontLeft,
      DriveConstants.TurningInverted.kFrontLeft
    );
    m_frontRight = new SwerveModule(
      "Front Right",
      DriveConstants.DriveCANIDs.kFrontRightDriving,
      DriveConstants.DriveCANIDs.kFrontRightTurning,
      DriveConstants.AngularOffsets.kFrontRight,
      m_table.getSubTable("SwerveModules"),
      DriveConstants.DrivingInverted.kFrontRight,
      DriveConstants.TurningInverted.kFrontRight
    );
    m_backLeft = new SwerveModule(
      "Back Left",
      DriveConstants.DriveCANIDs.kBackLeftDriving,
      DriveConstants.DriveCANIDs.kBackLeftTurning,
      DriveConstants.AngularOffsets.kBackLeft,
      m_table.getSubTable("SwerveModules"),
      DriveConstants.DrivingInverted.kBackLeft,
      DriveConstants.TurningInverted.kBackLeft
    );
    m_backRight = new SwerveModule(
      "Back Right",
      DriveConstants.DriveCANIDs.kBackRightDriving,
      DriveConstants.DriveCANIDs.kBackRightTurning,
      DriveConstants.AngularOffsets.kBackRight,
      m_table.getSubTable("SwerveModules"),
      DriveConstants.DrivingInverted.kBackRight,
      DriveConstants.TurningInverted.kBackRight
    );

    m_modules = new SwerveModule[] {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

    m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getGyroYaw(), getModulePositions(), new Pose2d());
    
    m_rotationController.enableContinuousInput(0, 2 * Math.PI); // TODO add pid constants for this

    m_magnitudeLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kMag);
    m_directionalLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kDir);
    m_rotationLimiter = new SlewRateLimiter(DriveConstants.SlewRate.kRot);
  }

  /* ----- OVERRIDES ----- */

  @Override
  public void periodic() {
    super.periodic();
    m_poseEstimator.update(getGyroYaw(), getModulePositions());
    updateVisionPoseEstimation();
  }

  public void dashboardInit() {
    SmartDashboard.putData("Reset Yaw",
      new InstantCommand((this::resetGyroYaw), this)
    );
  }

  public void dashboardPeriodic() {
    // NavX
    SmartDashboard.putBoolean("NavX Connected", m_gyro.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating", m_gyro.isCalibrating());
    SmartDashboard.putNumber("Yaw Degrees", getGyroYaw().getDegrees());
  }

  protected void publishInit() {}

  public void publishPeriodic() {}

  /* ----- AUTONOMOUS ----- */

  public void pathPlannerConfig() {
    AutoBuilder.configure(
      this::getPose, this::setPoseEstimator, this::getMeasuredChassisSpeeds,
      (speeds, ignore) -> setChassisSpeeds(speeds),
      DriveAutoConstants.autoDriveController,
      DriveAutoConstants.robotConfig,
      () -> {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
      },
      this
    );
  }

  /* ------ GYRO ------ */

  public Rotation2d getGyroYaw() {
    Angle angleMeasure = Degrees.of(
      m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0)
    );
    return new Rotation2d(angleMeasure);
  }

  public void resetGyroYaw() {
    m_gyro.zeroYaw(); 
  }

  public void setGyroOffset(Rotation2d offset) {
    m_gyro.setAngleAdjustment(offset.getDegrees());
  }

  /* ----- POSE ESTIMATION ----- */

  /**
   * Gets the position estimations and feeds them into the pose estimator.
   */
  public void updateVisionPoseEstimation() {
    List<Optional<EstimatedRobotPose>> estimatedPoses = subsystems.vision.getCameraEstimatedPoses();
    List<Optional<Matrix<N3, N1>>> stdDevs = subsystems.vision.getPoseStdDevs(estimatedPoses);

    for (int poseIndex = 0; poseIndex < estimatedPoses.size(); poseIndex++) {
      Optional<EstimatedRobotPose> poseOptional = estimatedPoses.get(poseIndex);
      Optional<Matrix<N3, N1>> stdDevsOptional = stdDevs.get(poseIndex);

      if(!poseOptional.isPresent())
        continue;
      
      EstimatedRobotPose pose = poseOptional.get();
      Matrix<N3, N1> stdDev = stdDevsOptional.get();

      addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, stdDev);   
    }
  }

  /**
   * Adds a position to the pose esitmate given a position and the confidence that that position is accurite.
   * 
   * @param pose
   * @param timestamp
   * @param stdDevs
   */
  public void addVisionMeasurement(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs) {
    pose = new Pose2d(pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));

    if(pose.getX() < 0 || pose.getY() < 0)
      return;
    if(
      pose.getX() > VisionConstants.kAprilTagLayout.getFieldLength() || 
      pose.getY() > VisionConstants.kAprilTagLayout.getFieldWidth()
    )
      return;

    m_poseEstimator.addVisionMeasurement(pose, timeStamp, stdDevs);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Pose3d getPose3d() {
    return new Pose3d(
      getPose().getMeasureX(),
      getPose().getMeasureY(),
      Meters.zero(),
      new Rotation3d(getPose().getRotation())
    );
  }

  /* ----- PATHING ----- */

  public PathPlannerPath getPath(Pose2d target) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      getPose(), target
    );

    // ChassisSpeeds measuredSpeeds = getMeasuredChassisSpeeds();
    // LinearVelocity measuredSpeed = Units.MetersPerSecond.of(
    //   Math.pow(measuredSpeeds.vxMetersPerSecond, 2) + 
    //   Math.pow(measuredSpeeds.vyMetersPerSecond, 2)
    // );

    PathPlannerPath path = new PathPlannerPath(
      waypoints, DriveAutoConstants.kPathingConstraints, null,
      new GoalEndState(0d, target.getRotation()) // figure out how this is different (holonomic rotation vs non holonomic rotation?)
    );

    return path;
  }

  public Command pathfindToCoralCommand(ReefSide reefSide, ReefBranch reefBranch, Distance tofCoralDisp) {
    Pose2d targetPose = reefSide
      .getEndPose(reefBranch)
      .plus(
        reefSide.kPerpNorm.times(
          tofCoralDisp.in(Units.Meters)
        )
      );

    return AutoBuilder.pathfindToPose(targetPose, DriveAutoConstants.kPathingConstraints);
  }

  public Command pathfindToStation() {
    PathPlannerPath path;
    
    try {
      path = PathPlannerPath.fromPathFile(DriveAutoConstants.AutoNames.kPathingToStation);
    }
    catch (Exception e) {
      return Commands.none();
    }

    return AutoBuilder.pathfindThenFollowPath(path, DriveAutoConstants.kPathingConstraints);
  }

  public Command getPathCommand(Pose2d target) {
    return AutoBuilder.followPath(getPath(target));
  }

  public Command getPathCommand(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  /* ----- SWERVE ----- */

  public void setPoseEstimator(Pose2d pose2d) {
    m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose2d);
  }

  public ChassisSpeeds getMeasuredChassisSpeeds() {
    ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    return chassisSpeeds;
  }
  
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the states of the modules given an array of module states.
   * 
   * @param swerveModuleStates Array of modules states
   */
  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.MaxVels.kTranslationalVelocity  
    );

    for (int i = 0; i < 4; i++)
      m_modules[i].setModuleState(swerveModuleStates[i]);
  }

  /**
   * 
   * @return
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++)
      moduleStates[i] = m_modules[i].getModuleState();
    return moduleStates;
  }

  /**
   * 
   * @return
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] moduleStates = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++)
      moduleStates[i] = m_modules[i].getModulePosition();
    return moduleStates;
  }
  
  /**
   * Drive the robot.
   * 
   * @param x The x velocity, unitless on the range 0 to 1
   * @param y The y velocity, unitless on the range -1 to 1
   * @param rot The rotational velocity, unitless on the range -1 to 1
   */
  public void drive(double x, double y, double rot) {
    setChassisSpeeds(getChassisSpeeds(x, y, rot));
  }

  /**
   * Edit a given chassis speed 
   * 
   * @return
   */
  public ChassisSpeeds getChassisSpeedOnRotationControl(double x, double y, Rotation2d rotationalSetpoint) {
    ChassisSpeeds chassisSpeeds = getChassisSpeeds(x, y, 0);
    
    return new ChassisSpeeds(
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond,
      m_rotationController.calculate(
        getGyroYaw().getRadians(), 
        rotationalSetpoint.getRadians()
      )
    );

  }

  /**
   * Given the drive controls return a chassis speed.
   * 
   * @param x
   * @param y
   * @param rot
   * @return
   */
  public ChassisSpeeds getChassisSpeeds(double x, double y, double rot) {

    x = Math.max(-1, Math.min(1, x));
    y = Math.max(-1, Math.min(1, y));
    rot = Math.max(-1, Math.min(1, rot));

    
    m_xVelocity = DriveConstants.MaxVels.kTranslationalVelocity.times(x);
    m_yVelocity = DriveConstants.MaxVels.kTranslationalVelocity.times(y);
    m_rotationalVelocity = DriveConstants.MaxVels.kRotationalVelocity.times(rot);

    // // convert to polar

    // double xv, yv, rv, mv, dv;
  
    // xv = DriveConstants.MaxVels.kTranslationalVelocity.times(x).in(MetersPerSecond);
    // yv = DriveConstants.MaxVels.kTranslationalVelocity.times(y).in(MetersPerSecond);
    // rv = DriveConstants.MaxVels.kRotationalVelocity.times(rot).in(RadiansPerSecond);
    
    // mv = Math.sqrt(Math.pow(xv, 2) + Math.pow(yv, 2));
    // dv = Math.atan2(yv, xv);

    // // rate limit

    // mv = m_magnitudeLimiter.calculate(mv);
    // dv = m_directionalLimiter.calculate(dv);

    // rv = m_rotationLimiter.calculate(rv);

    // // convert back and store units

    // xv = mv * Math.cos(dv);
    // yv = mv * Math.sin(dv);
    
    // m_xVelocity = MetersPerSecond.of(xv);
    // m_yVelocity = MetersPerSecond.of(xv);
    // m_rotationalVelocity = RadiansPerSecond.of(xv);

    // get chassis speeds

    ChassisSpeeds commandedChassisSpeeds;
    if (m_isFieldOriented){
      commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(m_xVelocity, m_yVelocity, m_rotationalVelocity, getGyroYaw());
    }
    else
      commandedChassisSpeeds = new ChassisSpeeds(m_xVelocity, m_yVelocity, m_rotationalVelocity);

    return commandedChassisSpeeds;
  }
}
