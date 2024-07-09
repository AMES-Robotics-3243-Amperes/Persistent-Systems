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

import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector {
    protected static final String configPath = "autoconfig.json";
    protected Map<String, Command> presets;
    protected ConfigData configData;

    public AutoSelector(Preset... presets) {
        this.presets = new HashMap<String, Command>();
        for (Preset preset: presets) {
            this.presets.put(preset.name, preset.command);
        }
    }

    public AutoSelector(Map<String, Command> presets) {
        this.presets = presets;
    }

    public void setSelectedPreset(String selectedPreset) {
        this.configData.selectedPreset = selectedPreset;
    }

    public void selectPreset() {
        JFrame frame = new JFrame("Select preset");
        frame.setLayout(new GridBagLayout());
        GridBagConstraints constraints = new GridBagConstraints();

        ScrollPane presetBox = new ScrollPane();
        JPanel selectionPanel = new JPanel();

        constraints.fill = GridBagConstraints.BOTH;
        constraints.gridx = 0;
        constraints.gridy = 0;
        constraints.weightx = 1;
        constraints.weighty = 9;
        frame.add(presetBox, constraints);

        Container selectionContainer = new Container();
        selectionContainer.setLayout(new BoxLayout(selectionContainer, BoxLayout.Y_AXIS));
        presetBox.add(selectionContainer);

        //selectionContainer.add(new TextArea("hi there"));

        ActionListeners.result = configData.selectedPreset;

        ButtonGroup buttonGroup = new ButtonGroup();
        for (String presetName: presets.keySet()) {
            JRadioButton button = new JRadioButton(presetName);
            buttonGroup.add(button);
            selectionContainer.add(button);
            if (presetName.equals(configData.selectedPreset)) {
                button.setEnabled(true);
            }

            button.addActionListener(new ActionListeners.OnRadioClick(presetName));
        }
        ActionListeners.shouldSave = false;
        ActionListeners.isDone = false;
        
        //selectionContainer.add(new TextArea("hi there"));

        constraints.fill = GridBagConstraints.HORIZONTAL;
        constraints.gridx = 0;
        constraints.gridy = 1;
        constraints.weightx = 1;
        constraints.weighty = 1;
        frame.add(selectionPanel, constraints);

        JButton done = new JButton("Done");
        done.addActionListener(ActionListeners::onDone);
        selectionPanel.add(done);
        JButton cancel = new JButton("Cancel");
        cancel.addActionListener(ActionListeners::onCancel);
        selectionPanel.add(cancel);

        frame.pack();
        frame.setVisible(true);

        while (frame.isVisible()) {
            if (ActionListeners.isDone)
                frame.setVisible(false);
        }

        if (ActionListeners.shouldSave) {
            configData.selectedPreset = ActionListeners.result;
            saveConfig();
        }

        frame.dispose();
    }

    protected static class ActionListeners {
        protected static boolean isDone = false;
        protected static boolean shouldSave = false;
        protected static String result = "";

        public static void onDone(ActionEvent e) {
            shouldSave = true;
            isDone = true;
        }

        public static void onCancel(ActionEvent e) {
            isDone = true;
        }

        public static class OnRadioClick implements ActionListener {
            private String output;

            public OnRadioClick(String output) {
                this.output = output;
            }

            @Override
            public void actionPerformed(ActionEvent e) {
                result = output;
            }
        }
    }

    public Command getAutoCommand() {
        return presets.get(configData.selectedPreset);
    }

    public void loadConfig() {
        FileReader configFileReader;
        try {
            configFileReader = new FileReader(configPath);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        try {


        configData = new JsonMapper().readValue(configFileReader, ConfigData.class);


        } catch (IOException e1) {
            try {
                configFileReader.close();
            } catch (IOException e2) {
                throw new RuntimeException(e2);
            }
            throw new RuntimeException(e1);
        }

        try {
            configFileReader.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public void saveConfig() {
        FileWriter configFileWriter;
        try {
            configFileWriter = new FileWriter(configPath);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        
        JSONObject configData = new JSONObject(this.configData);
        
        try {
            configFileWriter.write(configData.toString());
            configFileWriter.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public static class Preset {
        Command command;
        String name;

        public Preset(Command command, String name) {
            this.command = command;
            this.name = name;
        }
    }

    @JsonSerialize()
    protected static class ConfigData {
        public String selectedPreset;
    }
}
