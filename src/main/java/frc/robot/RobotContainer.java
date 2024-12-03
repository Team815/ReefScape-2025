// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.InputDevice;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.speedsmodulator.AccelerationLimiter;
import frc.robot.subsystems.speedsmodulator.AngleCorrector;
import frc.robot.subsystems.speedsmodulator.SpeedScaler;

public class RobotContainer {
    private final InputDevice controller = new XboxController();
    private double startingAngle;
    private Command resetHeadingCommand;
    private final AngleCorrector angleCorrector;
    private final SpeedScaler speedScaler;
    private String autoName;
    private final SwerveDrive drive;

    public RobotContainer() {
        final int frontLeftSpinId = 1;
        final int frontLeftRotateId = 2;
        final int frontRightSpinId = 7;
        final int frontRightRotateId = 8;
        final int backLeftSpinId = 5;
        final int backLeftRotateId = 6;
        final int backRightSpinId = 4;
        final int backRightRotateId = 3;
        final int frontLeftRotateSensorId = 1;
        final int frontRightRotateSensorId = 2;
        final int backLeftRotateSensorId = 3;
        final int backRightRotateSensorId = 4;
        final double frontLeftAngularOffset = 0.87d;
        final double frontRightAngularOffset = 0.57d;
        final double backLeftAngularOffset = 0.48d;
        final double backRightAngularOffset = 0.17d;
        final double maxDriveSpeed = 4.4d;

        // The max frame perimeter length is 120 in. For a square chassis,
        // each side would be 30 in. For safety, our chassis sides are 29 in.
        // Half of this is 14.5 in., or 0.368 m.

        final var halfLength = 0.368d;
        final var halfWidth = 0.368d;

        var moduleFrontLeft = SwerveModule.fromIds(
            frontLeftSpinId,
            frontLeftRotateId,
            frontLeftRotateSensorId,
            frontLeftAngularOffset,
            halfLength,
            halfWidth
        );

        var moduleFrontRight = SwerveModule.fromIds(
            frontRightSpinId,
            frontRightRotateId,
            frontRightRotateSensorId,
            frontRightAngularOffset,
            halfLength,
            -halfWidth);

        var moduleBackLeft = SwerveModule.fromIds(
            backLeftSpinId,
            backLeftRotateId,
            backLeftRotateSensorId,
            backLeftAngularOffset,
            -halfLength,
            halfWidth);

        var moduleBackRight = SwerveModule.fromIds(
            backRightSpinId,
            backRightRotateId,
            backRightRotateSensorId,
            backRightAngularOffset,
            -halfLength,
            -halfWidth);

        drive = new SwerveDrive(
            maxDriveSpeed,
            new Pigeon2(0),
            new SwerveModule[]{
                moduleFrontLeft,
                moduleFrontRight,
                moduleBackLeft,
                moduleBackRight});

        angleCorrector = new AngleCorrector(() -> drive.getAngle().getDegrees());
        drive.addModulator(angleCorrector);
        speedScaler = new SpeedScaler();
        drive.addModulator(speedScaler);
        var accelerationLimiter = new AccelerationLimiter(0.05d, 0.05d);
        drive.addModulator(accelerationLimiter);

        configureBindings();
    }

    private void configureBindings() {
        drive.setDefaultCommand(Commands.run(
            () -> drive.drive(new ChassisSpeeds(
                controller.getForwardVelocity(),
                controller.getSidewaysVelocity(),
                controller.getAngularVelocity())),
            drive));

        resetHeadingCommand = Commands.runOnce(() -> {
            drive.setAngle(180d);
            angleCorrector.reset(180d);
        }, drive);

        controller.resetHeading().onTrue(resetHeadingCommand);

        controller.resetHeadingForward().onTrue(Commands.runOnce(() -> resetHeading(0d)));
        controller.resetHeadingRight().onTrue(Commands.runOnce(() -> resetHeading(270d)));
        controller.resetHeadingBack().onTrue(Commands.runOnce(() -> resetHeading(180d)));
        controller.resetHeadingLeft().onTrue(Commands.runOnce(() -> resetHeading(90d)));
    }

    public Command getAutonomousCommand() {
        startingAngle = PathPlannerAuto
            .getStaringPoseFromAutoFile(autoName)
            .getRotation()
            .getDegrees();
        return new PathPlannerAuto(autoName).andThen(Commands.waitSeconds(15d));
    }

    public Command getTeleopInitCommand() {
        return Commands.runOnce(() -> {
            var angle = drive.getAngle().getDegrees() - startingAngle;
            drive.setAngle(angle);
            angleCorrector.reset(angle);
        });
    }

    private void resetHeading(double angle) {
        drive.setAngle(angle);
        angleCorrector.reset(angle);
    }
}
