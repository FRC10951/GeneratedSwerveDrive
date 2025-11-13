package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Elevator implements Subsystem {
    private final MotorController elevatorMotor;
    private final Encoder elevatorEncoder;

    private final double kP = 0.05;

    private double setpoint = 0.0;
    private double output = 0.0;

    private final double minHeight = 0.0;
    private final double maxHeight = 2.0;

    public Elevator(MotorController elevatorMotor, Encoder elevatorEncoder) {
        this.elevatorMotor = elevatorMotor;
        this.elevatorEncoder = elevatorEncoder;
    }

    public void setHeight(double targetHeight) {
        setpoint = Math.max(minHeight, Math.min(maxHeight, targetHeight));
    }

    public void stop() {
        elevatorMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double position = elevatorEncoder.getDistance();
        double error = setpoint - position;

        output = kP * error;

        output = Math.max(0.0, Math.min(1.0, output));

        elevatorMotor.set(error);
    }
}
