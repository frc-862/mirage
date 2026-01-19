package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.BooleanSupplier;

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
import frc.robot.constants.ShooterConstants;

public class MapleSim extends SubsystemBase {

    private final StructArrayPublisher<Pose3d> posePublisher;
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final SwerveDriveSimulation drivetrainSim;
    private final IntakeSimulation collectorSim;

    private BooleanSupplier isShooting = () -> false;
    private BooleanSupplier isCollecting = () -> false;
    private final Notifier shootNotifier;

    public MapleSim(Swerve drivetrain) {
        drivetrainSim = drivetrain.swerveSim.mapleSimDrive;
        collectorSim = IntakeSimulation.OverTheBumperIntake("Fuel", drivetrainSim,
            CollectorConstants.WIDTH, CollectorConstants.LENGTH_EXTENDED, IntakeSide.BACK, 5);


        arena.placeGamePiecesOnField();

        posePublisher = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("MapleSim")
            .getStructArrayTopic("Fuel", Pose3d.struct).publish();

        shootNotifier = new Notifier(this::shootFuel);
    }

    @Override
    public void simulationPeriodic(){
        posePublisher.set(arena.getGamePiecesPosesByType("Fuel").toArray(new Pose3d[0]));

        if (isCollecting.getAsBoolean()){
            collectorSim.startIntake();
        } else {
            collectorSim.stopIntake();
        }

        if (isShooting.getAsBoolean()){
            shootNotifier.startPeriodic(Seconds.of(0.2));
        } else {
            shootNotifier.stop();
        }
    }

    private void shootFuel(){
        collectorSim.obtainGamePieceFromIntake();

        arena.addGamePieceProjectile(new RebuiltFuelOnFly(
            drivetrainSim.getSimulatedDriveTrainPose().getTranslation(), ShooterConstants.SHOOTER_POSITION_ON_ROBOT,
            drivetrainSim.getDriveTrainSimulatedChassisSpeedsFieldRelative(), new Rotation2d(), ShooterConstants.SHOOTER_HEIGHT,
            MetersPerSecond.of(9), Degrees.of(60)));
    }

    public MapleSim withShooting(BooleanSupplier isShooting){
        if(isShooting.getAsBoolean()){
            this.isShooting = isShooting;
        }
        return this;
    }

    public MapleSim withCollecting(BooleanSupplier isCollecting){
        if(isCollecting.getAsBoolean()){
            this.isCollecting = isCollecting;
        }
        return this;
    }
}
