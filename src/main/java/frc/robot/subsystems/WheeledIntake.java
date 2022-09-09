package frc.robot.subsystems;

import org.strongback.components.Motor;
import frc.robot.interfaces.Intake;
import frc.robot.lib.Subsystem;

public class WheeledIntake extends Subsystem implements Intake {
    private Motor motor;
    private double currentOutput = 0;
    private boolean enabled;

    public WheeledIntake(Motor motor) {
        super("WheeledIntake");
        this.motor = motor;
        this.enabled = true;
    }

    @Override
    public String getName() {
        return "Wheeled Intake";
    }

    @Override
    public void enable() {
        enabled = true;
    }

    @Override
    public void disable() {
        setMotorOutput(0);
        enabled = false;
    }

    @Override
    public void execute(long timeInMillis) {}

    @Override
    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void cleanup() {
        setMotorOutput(0);
    }

    @Override
    public void setMotorOutput(double output) {
        if (output == currentOutput)
            return;
        currentOutput = output;
        motor.set(Motor.ControlMode.DutyCycle, output);
    }

    @Override
    public double getMotorOutput() {
        return currentOutput;
    }
}
