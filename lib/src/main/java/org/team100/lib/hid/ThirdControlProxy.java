package org.team100.lib.hid;

import org.team100.lib.async.Async;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Checks periodically for changes in the HID connected to port 2, and changes
 * the third control implementation to match.
 */
public class ThirdControlProxy implements ThirdControl {
    private static class NoThirdControl implements ThirdControl {
    }

    private static final int kPort = 2;
    private static final double kFreq = 1;

    private String m_name;
    private ThirdControl m_ThirdControl;

    public ThirdControlProxy(Async async) {
        refresh();
        async.addPeriodic(this::refresh, kFreq, "ThirdControlProxy");
    }

    public void refresh() {
        // name is blank if not connected
        String name = DriverStation.getJoystickName(kPort);
        if (name.equals(m_name))
            return;
        m_name = name;
        m_ThirdControl = getThirdControl(name);

        Util.printf("*** CONTROL UPDATE\n");
        Util.printf("* Third HID: %s Control: %s *\n",
                m_ThirdControl.getHIDName(),
                m_ThirdControl.getClass().getSimpleName());
    }

    private static ThirdControl getThirdControl(String name) {
        if (name.contains("MIDI")) {
            return new ThirdMidiControl();
        }
        if (name.contains("Buttons 2025")) {
            return new Buttons2025();
        }
        if (name.contains("Keyboard")) {
            return new Buttons2025();
        }
        return new NoThirdControl();
    }

    @Override
    public String getHIDName() {
        return m_ThirdControl.getHIDName();
    }

    @Override
    public boolean red1() {
        return m_ThirdControl.red1();
    }

    @Override
    public boolean red2() {
        return m_ThirdControl.red2();
    }

    @Override
    public boolean red3() {
        return m_ThirdControl.red3();
    }

    @Override
    public boolean red4() {
        return m_ThirdControl.red4();
    }

    @Override
    public boolean barge() {
        return m_ThirdControl.barge();
    }

    @Override
    public ScoringPosition scoringPosition() {
        return m_ThirdControl.scoringPosition();
    }

    @Override
    public boolean l1() {
        return m_ThirdControl.l1();
    }

    @Override
    public boolean l2() {
        return m_ThirdControl.l2();
    }

    @Override
    public boolean l3() {
        return m_ThirdControl.l3();
    }

    @Override
    public boolean l4() {
        return m_ThirdControl.l4();
    }

    @Override
    public boolean a() {
        return m_ThirdControl.a();
    }

    @Override
    public boolean b() {
        return m_ThirdControl.b();
    }

    @Override
    public boolean c() {
        return m_ThirdControl.c();
    }

    @Override
    public boolean d() {
        return m_ThirdControl.d();
    }

    @Override
    public boolean e() {
        return m_ThirdControl.e();
    }

    @Override
    public boolean f() {
        return m_ThirdControl.f();
    }

    @Override
    public boolean g() {
        return m_ThirdControl.g();
    }

    @Override
    public boolean h() {
        return m_ThirdControl.h();
    }

    @Override
    public boolean i() {
        return m_ThirdControl.i();
    }

    @Override
    public boolean j() {
        return m_ThirdControl.j();
    }

    @Override
    public boolean k() {
        return m_ThirdControl.k();
    }

    @Override
    public boolean l() {
        return m_ThirdControl.l();
    }

    @Override
    public boolean ab() {
        return m_ThirdControl.ab();
    }

    @Override
    public boolean cd() {
        return m_ThirdControl.cd();
    }

    @Override
    public boolean ef() {
        return m_ThirdControl.ef();
    }

    @Override
    public boolean gh() {
        return m_ThirdControl.gh();
    }

    @Override
    public boolean ij() {
        return m_ThirdControl.ij();
    }

    @Override
    public boolean kl() {
        return m_ThirdControl.kl();
    }

}
