package org.team100.lib.commands;

import java.util.function.BooleanSupplier;

import org.team100.lib.hid.ThirdControl;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Buttons2025Demo {
    ThirdControl m_control;

    public Buttons2025Demo(ThirdControl control) {
        m_control = control;
    }

    /** Binds all the buttons to print commands. */
    public void setup() {
        whileTrue(m_control::red1, new PrintCommand("red1"));
        whileTrue(m_control::red2, new PrintCommand("red2"));
        whileTrue(m_control::red3, new PrintCommand("red3"));
        whileTrue(m_control::red4, new PrintCommand("red4"));
        whileTrue(m_control::barge, new PrintCommand("barge"));
        whileTrue(m_control::l1, new PrintCommand("l1"));
        whileTrue(m_control::l2, new PrintCommand("l2"));
        whileTrue(m_control::l3, new PrintCommand("l3"));
        whileTrue(m_control::l4, new PrintCommand("l4"));
        whileTrue(m_control::a, new PrintCommand("a"));
        whileTrue(m_control::b, new PrintCommand("b"));
        whileTrue(m_control::c, new PrintCommand("c"));
        whileTrue(m_control::d, new PrintCommand("d"));
        whileTrue(m_control::e, new PrintCommand("e"));
        whileTrue(m_control::f, new PrintCommand("f"));
        whileTrue(m_control::g, new PrintCommand("g"));
        whileTrue(m_control::h, new PrintCommand("h"));
        whileTrue(m_control::i, new PrintCommand("i"));
        whileTrue(m_control::j, new PrintCommand("j"));
        whileTrue(m_control::k, new PrintCommand("k"));
        whileTrue(m_control::l, new PrintCommand("l"));
        whileTrue(m_control::ab, new PrintCommand("ab"));
        whileTrue(m_control::cd, new PrintCommand("cd"));
        whileTrue(m_control::ef, new PrintCommand("ef"));
        whileTrue(m_control::gh, new PrintCommand("gh"));
        whileTrue(m_control::ij, new PrintCommand("ij"));
        whileTrue(m_control::kl, new PrintCommand("kl"));
    }


    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }
}
