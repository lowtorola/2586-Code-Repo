// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ClimbConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final CANSparkMax m_leftTele = new CANSparkMax(LEFT_TELESCOPE, MotorType.kBrushless);
    private final CANSparkMax m_rightTele = new CANSparkMax(RIGHT_TELESCOPE, MotorType.kBrushless);

    private final DigitalInput m_leftLowLim = new DigitalInput(8);
    private final DigitalInput m_rightLowLim = new DigitalInput(9);

    private final RelativeEncoder m_leftEncoder;
    private final RelativeEncoder m_rightEncoder;

    private final DoubleSolenoid m_leftPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, PIVOT_LEFT[0], PIVOT_LEFT[1]);
    private final DoubleSolenoid m_rightPivot = new DoubleSolenoid(PneumaticsModuleType.REVPH, PIVOT_RIGHT[0], PIVOT_RIGHT[1]);

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    m_leftEncoder = m_leftTele.getEncoder();
    m_rightEncoder = m_rightTele.getEncoder();
     
    m_rightTele.setInverted(true);

    m_leftTele.restoreFactoryDefaults();
    m_rightTele.restoreFactoryDefaults();
    m_leftTele.setSmartCurrentLimit(40);
    m_rightTele.setSmartCurrentLimit(40);
    // m_leftTele.setSoftLimit(SoftLimitDirection.kForward, MAX_HEIGHT);
    // m_rightTele.setSoftLimit(SoftLimitDirection.kForward, MAX_HEIGHT);
    m_leftTele.setIdleMode(IdleMode.kBrake);
    m_rightTele.setIdleMode(IdleMode.kBrake);
  }

    public void retractTele() {
      m_leftTele.set(-WINCH_SPEED);
      m_rightTele.set(-WINCH_SPEED);
    }
    public void extendTele() {
      m_leftTele.set(WINCH_SPEED);
      m_rightTele.set(WINCH_SPEED);
    }
    public void stopTele() {
      m_leftTele.stopMotor();
      m_rightTele.stopMotor();
    }

    public void extendPivot() {
      m_leftPivot.set(Value.kForward);
      m_rightPivot.set(Value.kForward);
    }
    public void retractPivot() {
      m_leftPivot.set(Value.kReverse);
      m_rightPivot.set(Value.kReverse);
    }

    public void resetEncoders() {
      m_leftEncoder.setPosition(0);
      m_rightEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Tele Limit", m_leftLowLim.get());
    SmartDashboard.putBoolean("Right Tele Limit", m_rightLowLim.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
