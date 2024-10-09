package frc.robot.subsystems.notePipeline;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.NotePipelineConstants.CANIDs;

public class Shooter {
    private final CANSparkFlex m_shooterMotorL, m_shooterMotorR;
    private final SparkPIDController m_shooterPIDL, m_shooterPIDR;
    private final RelativeEncoder m_shooterEncoderL, m_shooterEncoderR;

    public Shooter() {
        m_shooterMotorL = new CANSparkFlex(CANIDs.kShooterLeft, MotorType.kBrushless);
        m_shooterEncoderL = m_shooterMotorL.getEncoder();
        m_shooterPIDL = m_shooterMotorL.getPIDController();
        m_shooterPIDL.setFeedbackDevice(m_shooterEncoderL);

        m_shooterMotorR = new CANSparkFlex(CANIDs.kShooterRight, MotorType.kBrushless);
        m_shooterEncoderR = m_shooterMotorR.getEncoder();
        m_shooterPIDR = m_shooterMotorR.getPIDController();
        m_shooterPIDR.setFeedbackDevice(m_shooterEncoderR);
    }
    
    /**
     * 
     * @param mps set the motors to a speed in meters per second
     */
    public void setMotor(double mps) {
        m_shooterPIDL.setReference(-mps, ControlType.kVelocity);
        m_shooterPIDR.setReference(mps, ControlType.kVelocity);
    } 
}
