package frc.robot.subsystems.algaeintake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;

import frc.robot.controller.Clamp;
import frc.robot.controller.CustomPIDController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Subsystem;
import frc.robot.constants.AlgaeIntakeConstants.AlgaePivotConstants;

public class AlgaePivot extends Subsystem {
    private final SparkMax m_algaePivotMotorController;

    private final AbsoluteEncoder m_algaePivotEncoder;
    private final CustomPIDController m_algaePivotPID;

    private double m_currentDesiredAngle = 0;

    protected static ArmSubsystem m_instance;
    public static ArmSubsystem getInstance() {
        if (m_instance == null)
            m_instance = new ArmSubsystem();
        return m_instance;
    }

    public AlgaePivot() {
        super();
        
        m_algaePivotMotorController = new SparkMax(
            AlgaePivotConstants.AlgaePivotCANIDs.kPivot,
            MotorType.kBrushless
        );
        
        m_algaePivotEncoder = m_algaePivotMotorController.getAbsoluteEncoder();

        m_algaePivotPID = new CustomPIDController(
            AlgaePivotConstants.PivotPID.pidConstants, 
            () -> getAngle().getRadians(),
            new Clamp<Double>(
                AlgaePivotConstants.kPivotMinPosition.getRadians(), 
                AlgaePivotConstants.kPivotMinPosition.getRadians()
            )
        );
    }

    public void setDesiredAngle(Rotation2d angle) {
        m_currentDesiredAngle = angle.getDegrees();
        m_algaePivotPID.setDesiredValue(angle.getRadians());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(m_algaePivotEncoder.getPosition());
    } 

    private void setMotor(double speed) {
        m_algaePivotMotorController.set(speed);
    }

    @Override
    public void periodic() {
        super.periodic();

        setMotor(m_algaePivotPID.calculate());
    }

    // Dashboard Fluff //
    protected void dashboardInit() {}

    protected void dashboardPeriodic() {
        SmartDashboard.putNumber("Current Angle", getAngle().getDegrees());
        SmartDashboard.putNumber("Desired Angle", m_currentDesiredAngle);
    }

    protected void publishInit() {}
    protected void publishPeriodic() {}
}
