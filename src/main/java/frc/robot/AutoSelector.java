package frc.robot;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.awt.Component;
import java.awt.Container;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.ScrollPane;
import java.awt.TextArea;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BoxLayout;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;

import org.json.JSONObject;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.json.JsonMapper;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    protected static final String configPath = "autoconfig.json"; // Currently unused
    protected Map<String, Command> presets;
    protected ConfigData configData; // Currently unused
    protected SendableChooser<Command> chooser;

    public AutoSelector() {
        presets = new HashMap<String, Command>();
        chooser = new SendableChooser<Command>();

        chooser.setDefaultOption("None", null);
        onUpdate();
    }

    public AutoSelector add(Command cmd, String name) {
        chooser.addOption(name, cmd);
        onUpdate();
        return this;
    }

    public AutoSelector add(Command cmd) {
        return add(cmd, cmd.getName());
    }

    public AutoSelector addDefault(Command cmd, String name) {
        chooser.setDefaultOption(name, cmd);
        onUpdate();
        return this;
    }

    public AutoSelector addDefault(Command cmd) {
        return addDefault(cmd, cmd.getName());
    }

    protected void onUpdate() {
        SmartDashboard.putData(chooser);
    }

    public Command get() {
        return chooser.getSelected();
    }

    @JsonSerialize() // Curently unused
    protected static class ConfigData {
        public String selectedPreset;
    }
}
