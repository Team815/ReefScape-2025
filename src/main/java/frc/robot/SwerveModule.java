package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public static final double DEFAULT_MAX_LINEAR_SPEED = 1d;
    public static final double DEFAULT_MAX_ANGULAR_SPEED = 0.2d;
    public static final double DEFAULT_MAX_LINEAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_MAX_ANGULAR_ACCELERATION = 0.1d;
    public static final double DEFAULT_P = 5d;

    private double maxLinearSpeed = DEFAULT_MAX_LINEAR_SPEED;
    private double maxAngularSpeed = DEFAULT_MAX_ANGULAR_SPEED;
    private final TalonFX linearMotor;
    private final TalonFX angularMotor;
    private final CANcoder angleSensor;
    private final Translation2d translation;
    private final PIDController pid = new PIDController(DEFAULT_P, 0d, 0d);

    public SwerveModule(
        TalonFX linearMotor,
        TalonFX angularMotor,
        CANcoder angleSensor,
        Translation2d translation) {
        this.linearMotor = linearMotor;
        this.angularMotor = angularMotor;
        this.angleSensor = angleSensor;
        this.translation = translation;
        pid.enableContinuousInput(-0.5d, 0.5d);
    }

    public static SwerveModule fromIds(
        int linearMotorId,
        int angularMotorId,
        int angleSensorId,
        double angleOffset,
        double x,
        double y) {
        var linearMotor = new TalonFX(linearMotorId);
        var angularMotor = new TalonFX(angularMotorId);
        setMaxAcceleration(linearMotor, DEFAULT_MAX_LINEAR_ACCELERATION);
        setMaxAcceleration(angularMotor, DEFAULT_MAX_ANGULAR_ACCELERATION);
        var angleSensor = new CANcoder(angleSensorId);
        var config = new MagnetSensorConfigs();
        config.MagnetOffset = angleOffset;
        var configurator = angleSensor.getConfigurator();
        configurator.apply(new CANcoderConfiguration());
        configurator.apply(config);
        return new SwerveModule(linearMotor, angularMotor, angleSensor, new Translation2d(x, y));
    }

    public void drive(SwerveModuleState state) {
        var angle = angleSensor.getAbsolutePosition().getValue();
        state = SwerveModuleState.optimize(
            state,
            Rotation2d.fromRotations(angle));
        var spinSpeed = MathUtil.clamp(state.speedMetersPerSecond, -maxLinearSpeed, maxLinearSpeed);
        linearMotor.set(spinSpeed);

        var rotation = state.angle.getRotations();
        pid.setSetpoint(rotation);
        var response = -pid.calculate(angle);
        angularMotor.set(MathUtil.clamp(response, -maxAngularSpeed, maxAngularSpeed));
    }

    public Translation2d getTranslation() {
        return translation;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            linearMotor.getPosition().getValue(),
            Rotation2d.fromRotations(angleSensor.getAbsolutePosition().getValue()));
    }

    private static void setMaxAcceleration(TalonFX motor, double maxAcceleration) {
    }
}
