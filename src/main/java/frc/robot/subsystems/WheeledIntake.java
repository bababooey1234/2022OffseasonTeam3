package frc.robot.subsystems;

import org.strongback.components.Motor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.Intake;
import frc.robot.lib.Subsystem;
import frc.robot.lib.chart.Chart;

public class WheeledIntake extends Subsystem implements Intake {
    private Motor motor;
    private double currentOutput = 0;
    private boolean enabled;

    public WheeledIntake(Motor motor) {
        super("WheeledIntake");
        this.motor = motor;
        this.enabled = true;

        Chart.register(() -> currentOutput, "%s/outputDutyCycle");
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

    /**
     * Update the operator console with the status of the hatch subsystem.
     */
    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Intake duty cycle", currentOutput);
    }
}
