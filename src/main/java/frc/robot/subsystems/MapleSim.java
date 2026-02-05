package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
    private boolean isShooting = false;

    public MapleSim(Swerve drivetrain, Collector collector, Indexer indexer, Turret turret, Hood hood, Shooter shooter) {
        this.collector = collector;
        this.indexer = indexer;
        this.turret = turret;
        this.hood = hood;
        this.shooter = shooter;

        drivetrainSim = drivetrain.swerveSim.mapleSimDrive;
        collectorSim = IntakeSimulation.OverTheBumperIntake("Fuel", drivetrainSim,
            Collector.CollectorConstants.WIDTH, Collector.CollectorConstants.LENGTH_EXTENDED, IntakeSide.FRONT, Collector.CollectorConstants.ROBOT_FUEL_CAPACITY);


        arena.placeGamePiecesOnField();

        posePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
            .getStructArrayTopic("Fuel", Pose3d.struct).publish();

        shootNotifier = new Notifier(this::shootFuel);
    }

    @Override
    public void simulationPeriodic(){
        posePublisher.set(arena.getGamePiecesPosesByType("Fuel").toArray(new Pose3d[0]));

        if (collector.getVelocity().gt(Collector.CollectorConstants.SIM_COLLECTING_THRESHOLD) && !collectorSim.isRunning()) {
            collectorSim.startIntake();
        } else if (collector.getVelocity().lt(Collector.CollectorConstants.SIM_COLLECTING_THRESHOLD) && collectorSim.isRunning()) {
            collectorSim.stopIntake();
        }

        if (indexer.getSpindexerVelocity().gt(Indexer.IndexerConstants.SIM_INDEX_THRESHOLD) && !isShooting) { // TODO: change to transfer when simulation merged
            shootNotifier.startPeriodic(Shooter.ShooterConstants.MAX_SHOOTING_PERIOD);
            isShooting = true;
        } else if (indexer.getSpindexerVelocity().lt(Indexer.IndexerConstants.SIM_INDEX_THRESHOLD) && isShooting) {
            shootNotifier.stop();
            isShooting = false;
        }
    }

    private void shootFuel(){
        if (collectorSim.obtainGamePieceFromIntake()) {
            arena.addGamePieceProjectile(new RebuiltFuelOnFly(
                drivetrainSim.getSimulatedDriveTrainPose().getTranslation(),
                Shooter.ShooterConstants.SHOOTER_POSITION_ON_ROBOT,
                drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                drivetrainSim.getSimulatedDriveTrainPose().getRotation().plus(new Rotation2d(turret.getAngle())),
                Shooter.ShooterConstants.SHOOTER_HEIGHT, MetersPerSecond.of(shooter.getLeftVelocity().in(RotationsPerSecond)
                * (Shooter.ShooterConstants.FLYWHEEL_CIRCUMFERENCE.in(Meters))),
                hood.getAngle()));
        }
    }
}
