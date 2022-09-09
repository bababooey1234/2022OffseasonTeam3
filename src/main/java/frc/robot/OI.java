package frc.robot;



import frc.robot.controller.Controller;
import frc.robot.controller.Sequence;
import frc.robot.controller.Sequences;
import frc.robot.drive.routines.*;
import frc.robot.interfaces.*;
import frc.robot.interfaces.Drivebase.DriveRoutineType;
import frc.robot.lib.GamepadButtonsX;
import frc.robot.lib.log.Log;
import frc.robot.subsystems.*;
import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.components.Clock;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Switch;
import org.strongback.components.ui.*;

public class OI {

    private SwitchReactor reactor = Strongback.switchReactor();
    private Controller controller;
    private Subsystems subsystems;

    public OI(Controller controller, Subsystems subsystems) {
        this.controller = controller;
        this.subsystems = subsystems;
    }

    /**
     * Configure the driver interface.
     * 
     * @param left the drivers left joystick
     * @param right the drivers right joystick
     */
    public void configureDriverJoysticks(FlightStick left, FlightStick right) {

        left.button(3).onPress(run(Sequences.constantDrivePower(-0.22)));

        // Arcade
        right.button(3).onPress(run(Sequences.setDrivebaseToDefault()));

    }

    /**
     * Configure the operators interface.
     * 
     * @param gamepad the operators joystick
     */
    public void configureOperatorJoystick(Gamepad gamepad) {



        gamepad.xButton().onPress(run(Sequences.stopConveyor()));
        gamepad.xButton().onRelease(run(Sequences.startConveyor()));

        gamepad.yButton().onPress(run(Sequences.stopConveyor()));
    }

    public void configureDDRPad(Dancepad dancepad) {}

    public void configureDiagBox(DiagnosticBox box) {

        // Conveyor overrides.
        // Get the interface that the diag box uses.
        // Setup the switch for manual/auto/off modes.
        // While the conveyor speed button is pressed, set the duty cycle. Does not turn
        // off.
    }

    /**
     * Registers all of the available drive routines that can be requested by the controller.
     */
    public void registerDriveRoutines(FlightStick left, FlightStick right,
            Dancepad dancepad) {
        // Convenience variables.
        Drivebase drivebase = subsystems.drivebase;
        Location location = subsystems.location;
        Clock clock = subsystems.clock;

        // Add the supported drive routines
        drivebase.registerDriveRoutine(DriveRoutineType.CONSTANT_POWER,
                new ConstantDrive("Constant Power", ControlMode.DutyCycle));
        drivebase.registerDriveRoutine(DriveRoutineType.CONSTANT_SPEED,
                new ConstantDrive("Constant Speed", ControlMode.Speed));

        drivebase.registerDriveRoutine(DriveRoutineType.ARCADE_CLIMB,
                new ArcadeClimb("ArcadeClimb", 1.0,
                        // Throttle.
                        left.getAxis(1).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 0.36 : 1),
                        // Turn power.
                        right.getAxis(0).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 0.36 : 1)));

        // The old favourite arcade drive with throttling if a button is pressed.
        drivebase.registerDriveRoutine(DriveRoutineType.ARCADE_DUTY_CYCLE,
                new ArcadeDrive("ArcadeDutyCycle", ControlMode.DutyCycle, 1.0,
                        // Throttle.
                        left.getAxis(1).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 0.36 : 1),
                        // Turn power.
                        right.getAxis(0).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 0.36 : 1)));

        // The old favourite arcade drive with throttling if a button is pressed but
        // using velocity mode.
        drivebase.registerDriveRoutine(DriveRoutineType.ARCADE_VELOCITY,
                new ArcadeDrive("ArcadeVelocity", ControlMode.Speed, Config.drivebase.maxSpeed,
                        // Throttle
                        left.getAxis(1).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 1 : 0.36),
                        // Turn power.
                        right.getAxis(0).invert().deadband(Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger().isTriggered() ? 1 : 0.36)));

        // DDR!
        if (dancepad.getButtonCount() > 0) {
            drivebase.registerDriveRoutine(DriveRoutineType.DDRPAD_DRIVE,
                    new ArcadeDrive("DDRPadDrive", ControlMode.DutyCycle, 1.0,
                            dancepad.getAxis(1).invert().scale(0.5), // Throttle.
                            dancepad.getAxis(0).invert().scale(0.4) // Turn power.
                    ));
        }

        // Cheesy drive.
        drivebase.registerDriveRoutine(DriveRoutineType.CHEESY,
                new CheesyDpadDrive("CheesyDPad", left.getDPad(0), // DPad
                        left.getAxis(GamepadButtonsX.LEFT_Y_AXIS), // Throttle
                        left.getAxis(GamepadButtonsX.RIGHT_X_AXIS), // Wheel (turn?)
                        left.getButton(GamepadButtonsX.RIGHT_TRIGGER_AXIS))); // Is quick turn

        // Drive through supplied waypoints using splines.
        drivebase.registerDriveRoutine(DriveRoutineType.TRAJECTORY,
                new TrajectoryDrive(location, clock));

        // Map joysticks in arcade mode for testing/tuning.
        // Roughly matches the speed of normal arcade drive.
        final double maxSpeed = Config.drivebase.maxSpeed;
        final double maxTurn = 0.75 * Config.drivebase.maxSpeed;
        drivebase.registerDriveRoutine(DriveRoutineType.POSITION_PID_ARCADE,
                new PIDDrive("PIDDrive",
                        () -> left.getAxis(1).invert().squarePreservingSign()
                                .scale(maxSpeed).read(),
                        () -> right.getAxis(0).squarePreservingSign().scale(maxTurn).read(),
                        drivebase, clock));
    }

    /**
     * Three position switch showing up as two buttons. Allows switching between
     * automatic, manual and disabled modes.
     * 
     * @param box the button box as a joystick.
     * @param colour the colour of the override switch.
     * @param subsystem the subystem to set the mode on.
     */
    /**
     * Changes the sequences mapped to buttons depending on a mode. The mode can be
     * enabled or disabled based on a button. An example would be to have an
     * intaking or shooting mode, where the buttons run different sequences
     * depending on which buttons are pressed. Turning on the mode is one button and
     * turning it off is another.
     * 
     * Example:
     * 
     * <pre>
     * {@code
     * onMode(rightStick.getButton(5), rightStick.getButton(6), "climb/drive",
     *         Sequences.enableClimbMode(), Sequences.enableDriveMode())
     *                 .onPress(rightStick.getButton(1), Sequences.releaseClimberRatchet(),
     *                         Sequences.startSlowDriveForward())
     *                 .onRelease(rightStick.getButton(2), Sequences.releaseClimberRatchet(),
     *                         Sequences.driveFast());
     * }
     * </pre>
     * 
     * Button 5 enabled climb mode, button 6 enables drive mode. If in climb mode,
     * buttons 1 and 2 run different sequences.
     * 
     * @param switchOn condition used to enable the mode. Normally a button
     *        press.
     * @param switchOff condition used to disable the mode. Normally a button
     *        press.
     * @param name used for logging when the mode changes.
     * @param activateSeq sequence to run when the mode is actived.
     * @param deactiveSeq sequence to run when the mode is deactived.
     * @return the ModeSwitch for further chaining of more buttons based on the
     *         mode.
     */
    @SuppressWarnings("unused")
    private ModeSwitch onMode(Switch switchOn, Switch switchOff, String name, Sequence activateSeq,
            Sequence deactiveSeq) {
        return new ModeSwitch(switchOn, switchOff, name, activateSeq, deactiveSeq);
    }

    /**
     * Changes the sequences mapped to buttons depending on a mode. The mode can be
     * enabled or disabled based on a button. An example would be to have an
     * intaking or shooting mode, where the buttons run different sequences
     * depending on which buttons are pressed. Turning on the mode is one button and
     * turning it off is another.
     */
    @SuppressWarnings("unused")
    private class ModeSwitch {
        private boolean active = false;

        /**
         * Creates a ModeSwitch to track the state and run sequences on state change.
         * 
         * @param switchOn condition used to enable the mode. Normally a button
         *        press.
         * @param switchOff condition used to disable the mode. Normally a button
         *        press.
         * @param name used for logging when the mode changes.
         * @param activatedSeq sequence to run when the mode is actived.
         * @param deactivedSeq sequence to run when the mode is deactived.
         * @return the ModeSwitch for chaining of more buttons based on the mode.
         */
        public ModeSwitch(Switch switchOn, Switch switchOff, String name, Sequence activatedSeq,
                Sequence deactivedSeq) {
            Strongback.switchReactor().onTriggered(switchOn, () -> {
                if (active) {
                    return;
                }
                Log.debug("Sequences", "Activating " + name);
                controller.run(activatedSeq);
                active = true;
            });
            Strongback.switchReactor().onTriggered(switchOff, () -> {
                if (!active) {
                    return;
                }
                Log.debug("Sequences", "Deactivating " + name);
                controller.run(deactivedSeq);
                active = false;
            });
        }

        /**
         * Run different sequences depending on the mode on button press.
         * 
         * @param swtch condition to trigger a sequence to run. Normally a button
         *        press.
         * @param activeSeq sequence to run if the mode is active.
         * @param inactiveSeq sequence to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch onPress(Switch swtch, Sequence activeSeq, Sequence inactiveSeq) {
            reactor.onTriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }

        /**
         * Run different sequences depending on the mode on button release.
         * 
         * @param swtch condition to trigger a sequence to run. Normally a button
         *        release.
         * @param activeSeq sequence to run if the mode is active.
         * @param inactiveSeq sequence to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch onRelease(Switch swtch, Sequence activeSeq, Sequence inactiveSeq) {
            reactor.onUntriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }

        /**
         * Run different sequences depending on the mode while a button is pressed.
         * 
         * @param swtch condition to trigger a sequence to run. Normally while a
         *        button is pressed.
         * @param activeSeq sequence to run if the mode is active.
         * @param inactiveSeq sequence to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch whileTriggered(Switch swtch, Sequence activeSeq, Sequence inactiveSeq) {
            reactor.whileTriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }
    }

    /**
     * Converts a sequence into something runnable. Returns the sequence name in
     * toString() for the code that prints out the button mappings in Trigger.
     * 
     * @param sequence the sequence to run.
     * @return Runnable that can be executed by the Trigger methods.
     */
    public Runnable run(Sequence sequence) {
        return new Runnable() {
            @Override
            public void run() {
                controller.run(sequence);
            }

            @Override
            public String toString() {
                return sequence.getName();
            }
        };

    }
}
