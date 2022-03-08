package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class HomeTelescopes extends CommandBase {

    private ClimbSubsystem m_climbSubsystem;
    private boolean leftFinished, rightFinished;

    public HomeTelescopes(ClimbSubsystem climbSubsystem) {
        this.m_climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        // Will be "true" or finished, when switch is pressed!
        leftFinished = m_climbSubsystem.getLeftLimit();
        // Will be "true" or finished, when switch is pressed!
        rightFinished = m_climbSubsystem.getRightLimit();
    }

    @Override
    public void execute() {
        
        leftFinished = m_climbSubsystem.getLeftLimit();
        rightFinished = m_climbSubsystem.getRightLimit();

        if(!leftFinished) {
            m_climbSubsystem.setLeftTele(-0.15); // FIXME: make sure this doesn't zip past limit
        } else {
            m_climbSubsystem.stopLeft();
        }
        if(!rightFinished) {
            m_climbSubsystem.setRightTele(-0.15); // FIXME: make sure this doesn't zip past limit
        } else {
            m_climbSubsystem.stopRight();
        }
    }

    @Override
    public boolean isFinished() {
        if(leftFinished && rightFinished) {
            m_climbSubsystem.resetEncoders();
            return true;
        } else {
            return false;
        }
    }

    
}
