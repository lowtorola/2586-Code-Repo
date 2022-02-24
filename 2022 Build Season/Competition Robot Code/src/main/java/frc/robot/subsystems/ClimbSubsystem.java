// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.ClimbConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    private final CANSparkMax m_leftTele = new CANSparkMax(LEFT_TELESCOPE, MotorType.kBrushless);
    private final CANSparkMax m_rightTele = new CANSparkMax(RIGHT_TELESCOPE, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
      //m_rightTele.follow(m_leftTele, true);
     //  m_leftTele.setSoftLimit(SoftLimitDirection.kForward, limit)
     m_rightTele.setInverted(true);
  }

    public void retractLeft() {
        m_leftTele.set(-WINCH_SPEED);
    }

    public void extendLeft() {
        m_leftTele.set(WINCH_SPEED);
    }

    public void retractRight() {
        m_rightTele.set(-WINCH_SPEED);
    }

    public void extendRight() {
        m_rightTele.set(WINCH_SPEED);
    }

    public void stopLeft() {
        m_leftTele.set(0);
    }

    public void stopRight() {
        m_rightTele.set(0);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
