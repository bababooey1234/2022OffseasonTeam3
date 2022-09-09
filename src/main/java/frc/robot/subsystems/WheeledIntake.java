package frc.robot.subsystems;

import frc.robot.interfaces.Intake;

public class WheeledIntake implements Intake {
    public WheeledIntake() {}

    @Override
    public String getName() {
        return "Wheeled Intake";
    }

    @Override
    public void enable() {}

    @Override
    public void disable() {}

    @Override
    public void execute(long timeInMillis) {}

    @Override
    public boolean isEnabled() {
        return true;
    }

    @Override
    public void cleanup() {}

    @Override
    public void setMotorOutput(double output) {}

    @Override
    public double getMotorOutput() {
        return 0;
    }
}
