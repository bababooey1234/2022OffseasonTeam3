/**
 * Sequences for doing most actions on the robot.
 * 
 * If you add a new sequence, add it to allSequences at the end of this file.
 */
package frc.robot.controller;

import static frc.robot.lib.PoseHelper.createPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Config;
import frc.robot.controller.Sequence.SequenceBuilder;
import frc.robot.lib.LEDColour;
import java.util.List;

/**
 * Control sequences for most robot operations.
 */
public class Sequences {

    /**
     * Do nothing sequence.
     */
    public static Sequence getEmptySequence() {
        if (emptySeq == null) {
            emptySeq = new SequenceBuilder("empty").build();
        }
        return emptySeq;
    }

    private static Sequence emptySeq = null;

    /**
     * The first sequence run in the autonomous period.
     */
    public static Sequence getStartSequence() {
        if (startSeq == null) {
            startSeq = new SequenceBuilder("start").build();
        }
        // startbuilder.add().doArcadeVelocityDrive();
        return startSeq;
    }

    private static Sequence startSeq = null;

    /**
     * Returns the sequence to reset the robot. Used to stop ejecting etc. The lift
     * is at intake height, the intake is stowed, all motors are off.
     * 
     * @return
     */
    public static Sequence getResetSequence() {
        if (resetSeq == null) {
            SequenceBuilder builder = new SequenceBuilder("empty");
            builder.then().doDefaultDrive();
            resetSeq = builder.build();
        }
        return resetSeq;
    }

    private static Sequence resetSeq = null;

    /**
     * Drive to a point on the field, relative to the starting point.
     * 
     * @param angle the final angle (relative to the field) in degrees.
     */
    public static Sequence getDriveToWaypointSequence(double x, double y, double angle) {
        Pose2d start = new Pose2d();
        Pose2d end = createPose2d(x, y, angle);
        SequenceBuilder builder = new SequenceBuilder(String.format("drive to %s", end));
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        driveToWaypointSeq = builder.build();
        return driveToWaypointSeq;
    }

    private static Sequence driveToWaypointSeq = null;

    public static Sequence setDrivebaseToArcade() {
        SequenceBuilder builder = new SequenceBuilder("Arcade Drive Routine");
        builder.then().doArcadeDrive();
        return builder.build();
    }

    public static Sequence setDrivebaseToDefault() {
        SequenceBuilder builder = new SequenceBuilder("Default Drive Routine");
        builder.then().doDefaultDrive();
        return builder.build();
    }

    public static Sequence startConveyor() {
        SequenceBuilder builder = new SequenceBuilder("Start conveyor");
        builder.then().setConveyorDutyCycle(Config.conveyor.dutyCycle);
        return builder.build();
    }

    public static Sequence reverseConveyor() {
        SequenceBuilder builder = new SequenceBuilder("Reverse conveyor");
        builder.then().setConveyorDutyCycle(-Config.conveyor.dutyCycle);
        return builder.build();
    }

    public static Sequence stopConveyor() {
        SequenceBuilder builder = new SequenceBuilder("Stop conveyor");
        builder.then().setConveyorDutyCycle(0);
        return builder.build();
    }

    public static Sequence constantDrivePower(double power) {
        SequenceBuilder builder = new SequenceBuilder("Constant drive power " + power);
        builder.then().setDrivebasePower(power);
        return builder.build();
    }


    public static Sequence setLEDColour(LEDColour c) {
        SequenceBuilder builder = new SequenceBuilder("set LEDS to " + c);
        builder.then().setColour(c);
        return builder.build();
    }

    public static Sequence startIntaking() {
        SequenceBuilder builder = new SequenceBuilder("start Intaking");
        builder.then().setMotorOutput(1);
        return builder.build();
    }

    public static Sequence stopIntaking() {
        SequenceBuilder builder = new SequenceBuilder("start Intaking");
        builder.then().setMotorOutput(0);
        return builder.build();
    }

    public static Sequence startOuttaking() {
        SequenceBuilder builder = new SequenceBuilder("start Intaking");
        builder.then().setMotorOutput(-1);
        return builder.build();
    }

    // For testing. Needs to be at the end of the file.
    public static Sequence[] allSequences = new Sequence[] {
            getEmptySequence(), getStartSequence(), getResetSequence(),
            startConveyor(), reverseConveyor(), startIntaking(), stopIntaking(), startOuttaking()
    };
}
