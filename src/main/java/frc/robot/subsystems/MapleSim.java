package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CollectorConstants;
import frc.robot.constants.IndexerConstants;
import frc.robot.constants.ShooterConstants;

public class MapleSim extends SubsystemBase {

    private final Collector collector;
    private final Indexer indexer;
    private final Turret turret;
    private final Hood hood;
    private final Shooter shooter;


    private final StructArrayPublisher<Pose3d> posePublisher;
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final SwerveDriveSimulation drivetrainSim;
    private final IntakeSimulation collectorSim;

    private final Notifier shootNotifier;

    public MapleSim(Swerve drivetrain, Collector collector, Indexer indexer, Turret turret, Hood hood, Shooter shooter) {
        this.collector = collector;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        drivetrainSim = drivetrain.swerveSim.mapleSimDrive;
        collectorSim = IntakeSimulation.OverTheBumperIntake("Fuel", drivetrainSim,
            CollectorConstants.WIDTH, CollectorConstants.LENGTH_EXTENDED, IntakeSide.BACK, 50);


        arena.placeGamePiecesOnField();

        posePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
            .getStructArrayTopic("Fuel", Pose3d.struct).publish();

        shootNotifier = new Notifier(this::shootFuel);
    }

    @Override
    public void simulationPeriodic(){
        posePublisher.set(arena.getGamePiecesPosesByType("Fuel").toArray(new Pose3d[0]));

        if (collector.getVelocity().gt(CollectorConstants.SIM_COLLECTING_THRESHOLD) && !collectorSim.isRunning()) {
            collectorSim.startIntake();
        } else if (collector.getVelocity().lt(CollectorConstants.SIM_COLLECTING_THRESHOLD) && collectorSim.isRunning()) {
            collectorSim.stopIntake();
        }

        if (indexer.getSpindexerVelocity().gt(IndexerConstants.SIM_INDEX_THRESHOLD)) { // TODO: change to transfer when simulation merged
            shootNotifier.startPeriodic(Seconds.of(0.2));
        } else if (indexer.getSpindexerVelocity().lt(IndexerConstants.SIM_INDEX_THRESHOLD)) {
            shootNotifier.stop();
        }
    }

    private void shootFuel(){
        if (collectorSim.obtainGamePieceFromIntake()) {
            
            arena.addGamePieceProjectile(new RebuiltFuelOnFly(
                drivetrainSim.getSimulatedDriveTrainPose().getTranslation(), ShooterConstants.SHOOTER_POSITION_ON_ROBOT,
                drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(), new Rotation2d(), ShooterConstants.SHOOTER_HEIGHT,
                MetersPerSecond.of(9), Degrees.of(60)));
        }
    }
}
