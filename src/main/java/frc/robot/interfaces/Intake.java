package frc.robot.interfaces;

import org.strongback.Executable;

public interface Intake extends Subsystem, Executable, DashboardUpdater {
    public void setMotorOutput(double output);

    public double getMotorOutput();
}
