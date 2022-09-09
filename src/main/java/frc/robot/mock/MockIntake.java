package frc.robot.mock;

import frc.robot.interfaces.Intake;

public class MockIntake implements Intake {
    public MockIntake() {}

    @Override
    public String getName() {
        return "Mock Intake";
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
