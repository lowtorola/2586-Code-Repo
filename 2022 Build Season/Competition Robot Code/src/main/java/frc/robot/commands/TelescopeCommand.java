package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class TelescopeCommand extends CommandBase {
    private final int m_teleState;
    private final ClimbSubsystem m_climber;
    /**
     * Creates a new telescope command for driving the two telescoping lifts in sync to a target position.
     * @param teleState The position to drive the telescope to: 0=fully retracted, 1=stowed to transfer to pivots, and 2=fully extended.
     * @param climber The climb subsystem to control with this command.
     */
    public TelescopeCommand(int teleState, ClimbSubsystem climber) {
        this.m_teleState = teleState;
        this.m_climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        switch(m_teleState) {
        case 0:
            m_climber.teleLow();
            break;
        case 1:
            m_climber.teleStage();
            break;
        case 2:
            m_climber.teleHigh();
            break;
        default:
            m_climber.teleStage();
        }
    }

    @Override
    public void execute() {
        // switch(m_teleState) {
        //     case 0:
        //         m_climber.teleLow();
        //         break;
        //     case 1:
        //         m_climber.teleStage();
        //         break;
        //     case 2:
        //         m_climber.teleHigh();
        //         break;
        //     default:
        //         m_climber.teleStage();
        //     }
    }

    @Override
    public boolean isFinished() {
        if(m_climber.getLeftController().atGoal()
           && m_climber.getRightController().atGoal()) {
               m_climber.teleStop();
               return true;
        } else {
               return false;
        }
    }
}
