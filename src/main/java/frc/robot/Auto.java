package frc.robot;

import static frc.robot.lib.PoseHelper.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controller.Controller;
import frc.robot.controller.Sequence;
import frc.robot.controller.Sequence.SequenceBuilder;
import frc.robot.controller.Sequences;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.AutoPaths;
import java.io.IOException;

/**
 * Handles auto routine selection.
 * 
 * Auto routines should be defined in Sequences.java
 */
public class Auto implements LogHelper {
    private SendableChooser<Sequence> autoProgram = new SendableChooser<Sequence>();

    public Auto() {
        addAutoSequences();
        initAutoChooser();
    }

    public void executedSelectedSequence(Controller controller) {
        Sequence seq = autoProgram.getSelected();
        info("Starting selected auto program %s", seq.getName());
        controller.run(seq);
    }

    private void addAutoSequences() {
        autoProgram.setDefaultOption("Nothing", Sequences.getEmptySequence());
        // add2Ball();
        // add3Ball();
        for (double seconds = 0.5; seconds <= 3; seconds += 0.5)
            for (double speed = -0.1; speed >= -1; speed -= 0.1)
                addSecondsBackwards(seconds, speed);
    }

    private void addSecondsBackwards(double seconds, double speed) {
        SequenceBuilder builder =
                new SequenceBuilder(Double.toString(seconds) + " Seconds Backwards @ "
                        + Double.toString(speed));
        builder.then().setDrivebaseSpeed(-1);
        builder.then().setDelayDelta(seconds);
        builder.then().setDrivebaseSpeed(0);
        builder.then().doDefaultDrive();
        autoProgram.addOption(
                Double.toString(seconds) + " Seconds Backwards @ " + Double.toString(speed),
                builder.build());
    }

    private void add2Ball() {
        SequenceBuilder builder = new SequenceBuilder("2 Ball");

        Pose2d start = createPose2d(8.7, 6.6, -99.9428561883);
        builder.then().setCurrentPostion(start);
        // A small air leak can slow down intake deploying and the compressor needs more time
        builder.then().setDelayDelta(3);
        try {
            builder.then().driveRelativeWaypoints(AutoPaths.k2ball);
        } catch (IOException e) {
            error("Failed to load path %s", AutoPaths.k2ball);
            autoProgram.addOption("FAILED - 2 Ball", builder.build());
            return;
        }
        builder.then().setDelayDelta(0.5);

        // Start shooting
        builder.then().setConveyorDutyCycle(Config.conveyor.dutyCycle);

        builder.then().setDelayDelta(5);
        // Robot is not entirely off the auto line, so we need to reverse a little bit
        builder.then().setDrivebasePower(-0.3);
        builder.then().setDelayDelta(0.5);
        builder.then().doDefaultDrive();
        autoProgram.addOption("2 Ball", builder.build());
    }

    private void add3Ball() {
        SequenceBuilder builder = new SequenceBuilder("3 Ball");

        Pose2d start = createPose2d(10.05, 5.52, -150.4612177404);
        builder.then().setCurrentPostion(start);
        builder.then().setDelayDelta(0.5);
        try {
            builder.then().driveRelativeWaypoints(AutoPaths.k3ballA);
        } catch (IOException e) {
            error("Failed to load path %s", AutoPaths.k3ballA);
            autoProgram.addOption("FAILED - 3 Ball A", builder.build());
            return;
        }

        // Start shooting
        builder.then().setConveyorDutyCycle(Config.conveyor.dutyCycle);

        builder.then().setDelayDelta(1.2);
        try {
            builder.then().driveRelativeWaypoints(AutoPaths.k3ballB);
        } catch (IOException e) {
            error("Failed to load path %s", AutoPaths.k3ballB);
            autoProgram.addOption("FAILED - 3 Ball B", builder.build());
            return;
        }
        builder.then().setDelayDelta(0.5); // Wait for ball to intake
        try {
            builder.then().driveRelativeWaypoints(AutoPaths.k3ballC);
        } catch (IOException e) {
            error("Failed to load path %s", AutoPaths.k3ballC);
            autoProgram.addOption("FAILED - 3 Ball C", builder.build());
            return;
        }

        // Start shooting
        builder.then().setConveyorDutyCycle(Config.conveyor.dutyCycle);

        builder.then().setDelayDelta(1);
        if (Config.auto.threeBall.returnToStart) {
            try {
                builder.then().driveRelativeWaypoints(AutoPaths.k3ballD);
            } catch (IOException e) {
                error("Failed to load path %s", AutoPaths.k3ballD);
                autoProgram.addOption("FAILED - 3 Ball D", builder.build());
                return;
            }
        }
        autoProgram.addOption("3 Ball", builder.build());
    }

    private void initAutoChooser() {
        SmartDashboard.putData("Auto program", autoProgram);
    }

    @Override
    public String getName() {
        return "Auto";
    }
}
