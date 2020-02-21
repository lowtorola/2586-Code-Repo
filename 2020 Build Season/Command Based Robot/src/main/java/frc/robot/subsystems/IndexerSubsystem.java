package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final Spark indexerLeft = new Spark(
        IndexerConstants.kIndexerLeftID);
    private final Spark indexerRight = new Spark(
        IndexerConstants.kIndexerRightID);

    public IndexerSubsystem() {

    }

    public void runIndexer() {
        indexerLeft.set(0.4);
        indexerRight.set(0.4);
    }

    public void stopIndexer() {
        indexerLeft.set(0);
        indexerRight.set(0);
    }

    @Override
    public void periodic() {

    }
}