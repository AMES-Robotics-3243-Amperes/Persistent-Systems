package frc.robot.test.networking;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.time.LocalDateTime;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import frc.robot.test.TestManager.TestResults;

// TODO - Make this auto-reconnect if the client disconnects
// TODO - Increase robustness from strange outputs or disconnects
// TODO - Add documentation

/**
 * Represents the connection to the driver station. Used
 * to send test information to the display program on the
 * driver station.
 * @author Hale Barber (H!)
 */
public class Workstation implements AutoCloseable {

    private enum ProtocolState {
        Disconnected,
        Holding,
        Running
    }
    private ProtocolState protocolState = ProtocolState.Disconnected;

    private final String GROUP_SELECTION_TERMINATOR = "END_SELECTION";
    private final String RESULTS_TERMINATOR = "END_RESULTS";
    private final String QUESTION_HEADER = "BEGIN_QUESTION";
    private final int CONNECTION_PORT = 5809;

    private ExecutorService executor = Executors.newSingleThreadExecutor();
    private ServerSocket server;
    private Socket client;

    public Workstation() {
        System.out.println("TCP Server Started");
        try {
            server = new ServerSocket(CONNECTION_PORT);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        
        executor.execute(this::findConnection);
    }

    public synchronized Future<boolean[]> getChosenTestGroups(String[] testGroups) {
        verifyConnection();
        verifyProtocolState(ProtocolState.Holding);
        final PrintWriter writer = getWriter();
        final BufferedReader reader = getReader();
        
        return executor.submit(() -> {
            // Send all names, as per protocol
            for (String testGroupName : testGroups) {
                writer.println(testGroupName);
            }
            writer.println(GROUP_SELECTION_TERMINATOR);
            System.out.println("Groups sent");

            String response = reader.readLine();
            System.out.println("Response received: " + response);
            char[] responseChars = response.toCharArray();
            boolean[] out = new boolean[testGroups.length];

            for (int i = 0; i < responseChars.length; i++) {
                switch (responseChars[i]) {
                    case 'T':
                        out[i] = true;
                        break;
                    case 'F':
                        out[i] = false;
                        break;
                    default:
                        throw new IllegalStateException("Client gave an invalid response char to selection request, \'" + responseChars[i] + "\'.");
                }
            }

            synchronized (this) {
                protocolState = ProtocolState.Running;
            }

            return out;
        });
    }

    public synchronized Future<Boolean> askQuestion(String question, String trueOption, String falseOption) {
        verifyConnection();
        verifyProtocolState(ProtocolState.Running);
        final PrintWriter writer = getWriter();
        final BufferedReader reader = getReader();

        return executor.submit(() -> {
            writer.println(QUESTION_HEADER);
            writer.println(question);
            writer.println(trueOption);
            writer.println(falseOption);
            
            char response = reader.readLine().charAt(0);

            switch (response) {
                case 'T':
                    return true;
                case 'F':
                    return false;
                default:
                    throw new IllegalStateException("Client gave an invalid response char to question, \'" + response + "\'.");
            }
        });
    }

    public synchronized Future<?> publishResults(Map<String, Map<String, TestResults>> results) {
        verifyConnection();
        verifyProtocolState(ProtocolState.Running);
        final PrintWriter writer = getWriter();

        return executor.submit(() -> {
            for (Entry<String, Map<String, TestResults>> testGroup : results.entrySet()) {
                writer.println("G:" + testGroup.getKey());

                for (Entry<String, TestResults> test : testGroup.getValue().entrySet()) {
                    String prefix = "0:";
                    switch (test.getValue().m_successResult) {
                        case SUCCESS:
                            prefix = "S:";
                            break;
                        case FAIL:
                            prefix = "F:";
                            break;
                        case NOTRUN:
                            prefix = "N:";
                            break;
                    }

                    writer.println(prefix + test.getKey());
                    writer.println(test.getValue().m_message);
                }
            }

            writer.println(RESULTS_TERMINATOR);

            synchronized (this) {
                protocolState = ProtocolState.Holding;
            }
        });
    }

    private void findConnection() {
        try {
            Socket socket = server.accept();
            synchronized (this) {
                protocolState = ProtocolState.Holding;
                client = socket;
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private void verifyConnection() {
        if (!hasConnection()) {
            System.out.println(LocalDateTime.now());
            throw new IllegalStateException("No client Ammeter program has been connected. Please start one on the driver station and try again.");
        }
    }

    private void verifyProtocolState(ProtocolState correct) {
        if (protocolState != correct) {throw new IllegalStateException("Protocol dictates this action cannot be performed currently.");}
    }

    private PrintWriter getWriter() {
        try {
            return new PrintWriter(client.getOutputStream(), true);
        } catch (IOException e) {
            throw new IllegalStateException("No client Ammeter program has been connected. Please start one on the driver station and try again.");
        }
    }

    private BufferedReader getReader() {
        try {
            return new BufferedReader(new InputStreamReader(client.getInputStream()));
        } catch (IOException e) {
            throw new IllegalStateException("No client Ammeter program has been connected. Please start one on the driver station and try again.");
        }
    }

    public synchronized boolean hasConnection() {
        return client != null && !client.isClosed();
    }

    @Override
    public void close() {
        executor.shutdownNow();
        try {
            if (client != null) client.close();
            if (server != null) server.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        protocolState = ProtocolState.Disconnected;
    }
}
