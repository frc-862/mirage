// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerDrive extends Command {
    DoubleSupplier xInput;
    DoubleSupplier yInput;
    DoubleSupplier rInput;
    BooleanSupplier isFieldCentric;
    BooleanSupplier isSlowMode;
    Swerve drivetrain;

    Vector<N2> translation;
    double rot;
    SwerveRequest request;

    /**
     * applies deadbands, exponential scaling, and slow mode to controller inputs
     * @param xInput
     * @param yInput
     * @param rInput
     * @param isFieldCentric
     * @param isSlowMode
     * @param drivetrain
     */
    public ControllerDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput,
        BooleanSupplier isFieldCentric, BooleanSupplier isSlowMode, Swerve drivetrain) {
            this.xInput = xInput;
            this.yInput = yInput;
            this.rInput = rInput;
            this.isFieldCentric = isFieldCentric;
            this.isSlowMode = isSlowMode;
            this.drivetrain = drivetrain;

            addRequirements(drivetrain);
    }

    /**
     * applies deadbands and exponential scaling
     * @param xInput
     * @param yInput
     * @param rInput
     * @param drivetrain
     */
    public ControllerDrive(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier rInput, Swerve drivetrain) {
        this(xInput, yInput, rInput, () -> true, () -> false, drivetrain);
    }

    @Override
    public void execute() {
        translation = new Vector<>(new SimpleMatrix(new double[] {xInput.getAsDouble(), yInput.getAsDouble()}));
        translation = MathUtil.applyDeadband(translation, ControllerConstants.DEADBAND);
        translation = MathUtil.copyDirectionPow(translation, ControllerConstants.POW);
        translation = translation.times(isSlowMode.getAsBoolean() ? ControllerConstants.SLOW_MODE_MULT : 1);

        rot = rInput.getAsDouble();
        rot = MathUtil.applyDeadband(rot, ControllerConstants.DEADBAND);
        rot = MathUtil.copyDirectionPow(rot, ControllerConstants.POW);
        rot *= isSlowMode.getAsBoolean() ? ControllerConstants.SLOW_MODE_MULT : 1;

        if (isFieldCentric.getAsBoolean()) {
            request = new SwerveRequest.FieldCentric()
                .withVelocityX(DriveConstants.MaxSpeed.times(translation.get(0)))
                .withVelocityY(DriveConstants.MaxSpeed.times(translation.get(1)))
                .withRotationalRate(DriveConstants.MaxAngularRate.times(rot))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);

        } else {
            request = new SwerveRequest.RobotCentric()
                .withVelocityX(DriveConstants.MaxSpeed.times(translation.get(0)))
                .withVelocityY(DriveConstants.MaxSpeed.times(translation.get(1)))
                .withRotationalRate(DriveConstants.MaxAngularRate.times(rot))
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        }

        drivetrain.setControl(request);
    }
}
