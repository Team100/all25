package org.team100.lib.indicator;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.util.Takt;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;

/**
 * An LED strip used as a signal light.
 * 
 * Uses the AddressableLED feature of the RoboRIO.
 * 
 * We use these strips: https://www.amazon.com/gp/product/B01CNL6LLA
 * 
 * Note these strips use a different order: red-blue-green, not
 * red-green-blue, so the colors need some fixing up.
 */
public class LEDIndicator {

    /** Turns flashing on or off, in case you find it annoying. */
    private static final boolean kFlash = false;

    /** Maps indicator colors to WS2811 colors. */
    public enum State {

        BLACK(Color.kBlack),
        RED(Color.kRed),
        BLUE(Color.kBlue),
        GREEN(Color.kLime),
        PURPLE(Color.kFuchsia),
        YELLOW(Color.kYellow),
        ORANGE(new Color(1.0f, 0.1f, 0.0f)),
        WHITE(Color.kBlack); // turn off white to save battery

        /**
         * This "color" is what we tell the LED strip to make it display the actual
         * desired color.
         */

        private final Color color;

        /**
         * @param color the correct RGB color
         */
        private State(Color color) {
            if (RobotBase.isSimulation()) {
                // use RGB colors
                this.color = color;
            } else {
                // swap blue and green to make RBG
                this.color = new Color(color.red, color.blue, color.green);
            }
        }
    }

    /**
     * Fast flashing, 15hz.
     */
    private static final double kFlashDurationSec = 0.2;

    private final AddressableLED m_led;
    // 4/2/25 avoid the loop setting all the pixels
    // private final AddressableLEDBuffer m_buffer;
    private final AddressableLEDBuffer m_greenBuffer;
    private final AddressableLEDBuffer m_redBuffer;

    private final AddressableLEDBuffer m_orangeBuffer;
    private final AddressableLEDBuffer m_beigeBuffer;

    private final List<LEDStrip> m_frontStrips;
    private final List<LEDStrip> m_backStrips;
    private final Supplier<Boolean> m_isUsingBargeAsisst;
    private State m_front;
    private State m_back;
    private boolean m_flashing;

    private Supplier<Double> m_timeSinceLastPose;

    SwerveDriveSubsystem m_drive;
    public LEDIndicator(int port, Supplier<Double> timeSinceLastPose, Supplier <Boolean> isUsingBargeAssist, SwerveDriveSubsystem drive) {
        m_frontStrips = new ArrayList<>();
        m_backStrips = new ArrayList<>();
        m_drive = drive;
        m_isUsingBargeAsisst = isUsingBargeAssist;

        m_frontStrips.add(new LEDStrip(0, 21));
        m_backStrips.add(new LEDStrip(21, 40));

        int length = Math.max(
                m_frontStrips.stream().map(LEDStrip::end).reduce(0, Integer::max),
                m_backStrips.stream().map(LEDStrip::end).reduce(0, Integer::max));

        m_led = new AddressableLED(port);
        m_led.setLength(length);

        // m_buffer = new AddressableLEDBuffer(length);

        // buffer flipping is a little quicker than setting the pixels one at a time
        m_greenBuffer = new AddressableLEDBuffer(length);

        for (LEDStrip strip : m_frontStrips) {
            strip.solid(m_greenBuffer, Color.kGreen);
        }
        for (LEDStrip strip : m_backStrips) {
            strip.solid(m_greenBuffer, Color.kGreen);
        }

        m_redBuffer = new AddressableLEDBuffer(length);
        for (LEDStrip strip : m_frontStrips) {
            strip.solid(m_redBuffer, Color.kRed);
        }

        for (LEDStrip strip : m_backStrips) {
            strip.solid(m_redBuffer, Color.kRed);
        }

        m_orangeBuffer = new AddressableLEDBuffer(length);

        for (LEDStrip strip : m_frontStrips) {
            strip.solid(m_orangeBuffer, Color.kOrange);
        }

        for (LEDStrip strip : m_backStrips) {
            strip.solid(m_orangeBuffer, Color.kOrange);
        }

        m_beigeBuffer = new AddressableLEDBuffer(length);

        for (LEDStrip strip : m_frontStrips) {
            strip.solid(m_beigeBuffer, Color.kBeige);
        }

        for (LEDStrip strip : m_backStrips) {
            strip.solid(m_beigeBuffer, Color.kBeige);
        }


        m_led.setData(m_redBuffer);
        // m_led.setData(m_buffer);
        m_led.start();

        m_timeSinceLastPose = timeSinceLastPose;

        m_flashing = false;

    }

    public void setFront(State s) {
        m_front = s;
    }

    public void setBack(State s) {
        m_back = s;
    }

    public void setFlashing(boolean flashing) {
        m_flashing = flashing;
    }

    /**
     * Periodic does all the real work in this class.
     */
    public void periodic() {

        if(!m_isUsingBargeAsisst.get()){
            if(m_timeSinceLastPose.get() < 1){
                m_led.setData(m_greenBuffer);
            } else {
                m_led.setData(m_redBuffer);
            }
        } else{
            // if( Math.abs(m_drive.getPose().getX() - 7.2) < 0.05){
            //     m_led.setData(m_orangeBuffer);
            // } else {
            //     m_led.setData(m_beigeBuffer);
            // }
        }

        
    }

    public void close() {
        m_led.close();
    }
}
