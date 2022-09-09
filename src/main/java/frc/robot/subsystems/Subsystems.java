package frc.robot.subsystems;



import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Config;
import frc.robot.interfaces.*;
import frc.robot.lib.LEDColour;
import frc.robot.lib.MotorFactory;
import frc.robot.lib.NavXGyroscope;
import frc.robot.mock.*;
import org.strongback.Executor.Priority;
import org.strongback.Strongback;
import org.strongback.components.Clock;
import org.strongback.components.Gyroscope;
import org.strongback.components.Motor;
import org.strongback.components.PneumaticsModule;
import org.strongback.components.ui.InputDevice;
import org.strongback.hardware.Hardware;
import org.strongback.mock.Mock;

/**
 * Contains the subsystems for the robot.
 * 
 * Makes it easy to pass all subsystems around.
 */
public class Subsystems implements DashboardUpdater, LogHelper {
    // Not really a subsystem, but used by all subsystems.
    public Clock clock;
    public LEDStrip ledStrip;
    public Location location;
    public Drivebase drivebase;
    public PneumaticsModule pcm;
    public Monitor monitor;
    public InputDevice gamepad;
    public Intake intake;

    public Subsystems(Clock clock, InputDevice gamepad) {
        this.clock = clock;
        this.gamepad = gamepad;
    }

    public void enable() {
        info("Enabling subsystems");
        gamepad.setRumbleLeft(0);
        gamepad.setRumbleRight(0);
        drivebase.enable();
    }

    public void disable() {
        info("Disabling Subsystems");
        gamepad.setRumbleLeft(0);
        gamepad.setRumbleRight(0);
        drivebase.disable();
    }

    @Override
    public void updateDashboard() {
        drivebase.updateDashboard();
        location.updateDashboard();
    }

    /**
     * Create the drivebase subsystem. Creates the motors.
     */
    public void createDrivebase() {
        if (!Config.drivebase.present) {
            debug("Using mock drivebase");
            drivebase = new MockDrivebase();
            return;
        }
        Motor leftMotor = MotorFactory.getDriveMotor(true, clock);
        Motor rightMotor = MotorFactory.getDriveMotor(false, clock);

        leftMotor.setPosition(0);
        rightMotor.setPosition(0);
        try {
            // Let the encoders get the message and have time to send it back to us.
            Thread.sleep(100);
        } catch (InterruptedException e) {
        }
        error("Reset drive encoders to zero, currently are: %f, %f", leftMotor.getPosition(),
                rightMotor.getPosition());
        // metres.
        drivebase =
                new DrivebaseImpl(leftMotor, rightMotor);
        Strongback.executor().register(drivebase, Priority.HIGH);
    }

    /**
     * Create the location subsystem. Creates the gyro.
     */
    public void createLocation() {
        if (!Config.drivebase.present) {
            debug("No drivebase, using mock location");
            location = new MockLocation();
            return;
        }
        Gyroscope gyro = new NavXGyroscope("NavX", Config.navx.present);
        gyro.zero();
        // Encoders must return metres.
        location = new LocationImpl(drivebase, gyro, clock);
        Strongback.executor().register(location, Priority.HIGH);
    }

    public void createLEDStrip() {
        if (!Config.ledStrip.present) {
            ledStrip = new MockLEDStrip();
            debug("LED Strip not present, using a mock LED Strip instead.");
            return;
        }
        ledStrip = new LEDStripImpl(Config.ledStrip.pwmPort, Config.ledStrip.numLEDs);
    }

    public void createMonitor() {
        monitor = new MonitorImpl();
    }

    public void createWheeledIntake(){
        intake = new WheeledIntake(motor)
    }

    public void setLEDFinalCountdown(double time) {
        ledStrip.setProgressColour(LEDColour.RED, LEDColour.GREEN,
                1 - (time / Config.ledStrip.countdown));
    }

    /**
     * Create the Pneumatics Control Module (PCM) subsystem.
     */
    public void createPneumatics() {
        if (!Config.pcm.present) {
            pcm = Mock.pneumaticsModule(Config.pcm.canId);
            debug("Created a mock compressor");
            return;
        }
        pcm = Hardware.pneumaticsModule(Config.pcm.canId, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public String getName() {
        return "Subsystems";
    }
}
