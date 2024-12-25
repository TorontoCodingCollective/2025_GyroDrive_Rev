package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightsConstants;

public class LightsSubsystem extends SubsystemBase {

    private final AddressableLED              ledStrip  = new AddressableLED(LightsConstants.LED_STRING_PWM_PORT);
    private final AddressableLEDBuffer        ledBuffer = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    // RSL Flash
    private static final Color                RSL_COLOR = new Color(255, 20, 0);
    private static final AddressableLEDBuffer RSL_ON    = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);
    private static final AddressableLEDBuffer RSL_OFF   = new AddressableLEDBuffer(LightsConstants.LED_STRING_LENGTH);

    static {
        // Set all the values in the buffer to the RSL colour
        LEDPattern.solid(RSL_COLOR).applyTo(RSL_ON);
        LEDPattern.kOff.applyTo(RSL_OFF);
    }

    private int     rslFlashCount    = -1;
    private boolean previousRslState = false;

    public LightsSubsystem() {

        ledStrip.setLength(60);
        ledStrip.start();
    }

    public void setDriveMotorSpeeds(double leftSpeed, double rightSpeed) {
        // FIXME: maybe light some LEDs
    }

    public void setRSLFlashCount(int count) {
        rslFlashCount = count;
    }

    @Override
    public void periodic() {

        if (rslFlashCount >= 0) {
            flashRSL();
        }
        else {
            ledStrip.setData(ledBuffer);
        }
    }

    /**
     * Flash all LEDs in the buffer in time with the RSL light for a set number of flashes
     */
    private void flashRSL() {

        boolean rslState = RobotController.getRSLState();

        // when the RSL goes from on to off, decrement the flash count
        if (!rslState && previousRslState) {
            rslFlashCount--;
        }
        previousRslState = RobotController.getRSLState();

        if (rslState) {
            ledStrip.setData(RSL_ON);
        }
        else {
            ledStrip.setData(RSL_OFF);
        }
    }

}
