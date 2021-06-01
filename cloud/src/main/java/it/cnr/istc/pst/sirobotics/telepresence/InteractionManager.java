package it.cnr.istc.pst.sirobotics.telepresence;

import java.io.IOException;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.stream.Collectors;

import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.sirobotics.telepresence.api.LongArray;
import it.cnr.istc.pst.sirobotics.telepresence.api.StringWrapper;

public class InteractionManager {

    private static final Logger LOG = LoggerFactory.getLogger(InteractionManager.class);
    private static final Executor COMMAND_EXECUTOR = Executors.newSingleThreadExecutor();
    private final Map<String, String> context = new HashMap<>();
    static final Map<String, Action> ACTIONS = new HashMap<>();
    private final Random random = new Random();
    private final String prefix;

    public InteractionManager(final String prefix) {
        this.prefix = prefix;
        try {
            App.MQTT_CLIENT.subscribe(prefix + "/user_utterance", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    try {
                        final StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                                StringWrapper.class);
                        final JsonNode intent = App.NLU_CLIENT.parse(string_message.data);
                        updateContext(Collections.singletonMap("intent", intent.get("intent").toString()));
                        executeActions();
                    } catch (IOException e) {
                        LOG.error("Cannot process the input text", e);
                    }
                });
            });
            App.MQTT_CLIENT.subscribe(prefix + "/CognitiveExercize", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    try {
                        LOG.info("We can now start the cognitive exercize {}" + message);
                        CognitiveExercizes ces = App.MAPPER.readValue(new String(message.getPayload()),
                                CognitiveExercizes.class);
                        CognitiveExercize ce = ces.atoms.iterator().next();
                        Map<String, String> ctx = new HashMap<>();
                        ctx.put("command", "CognitiveExercize");
                        ctx.put("sigma", Long.toString(ce.sigma));
                        ctx.put("id", Long.toString(ce.ex_id));
                        updateContext(ctx);
                        executeActions();
                    } catch (IOException e) {
                        LOG.error("Cannot process the cognitive exercise", e);
                    }
                });
            });
            App.MQTT_CLIENT.subscribe(prefix + "/Reminder", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    try {
                        LOG.info("We can now send the reminder {}" + message);
                        Reminders res = App.MAPPER.readValue(new String(message.getPayload()), Reminders.class);
                        Reminder rem = res.atoms.iterator().next();
                        Map<String, String> ctx = new HashMap<>();
                        ctx.put("command", "Reminder");
                        ctx.put("sigma", Long.toString(rem.sigma));
                        ctx.put("id", Long.toString(rem.rem_id));
                        updateContext(ctx);
                        executeActions();
                    } catch (IOException e) {
                        LOG.error("Cannot process the cognitive exercise", e);
                    }
                });
            });
            App.MQTT_CLIENT.subscribe(prefix + "/PhysiologicalMeasurement", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    try {
                        LOG.info("We can now ask for the physiological measurement {}" + message);
                        PhysiologicalMeasurements pms = App.MAPPER.readValue(new String(message.getPayload()),
                                PhysiologicalMeasurements.class);
                        PhysiologicalMeasurement pm = pms.atoms.iterator().next();
                        Map<String, String> ctx = new HashMap<>();
                        ctx.put("command", "PhysiologicalMeasurement");
                        ctx.put("sigma", Long.toString(pm.sigma));
                        ctx.put("id", Long.toString(pm.pm_id));
                        updateContext(ctx);
                        executeActions();
                    } catch (IOException e) {
                        LOG.error("Cannot process the cognitive exercise", e);
                    }
                });
            });
        } catch (MqttException ex) {
            LOG.error("Cannot subscribe the interaction manager #" + prefix + " to the MQTT broker..", ex);
        }
    }

    public boolean hasContextVariable(final String variable) {
        return context.containsKey(variable);
    }

    public String getContextValue(final String variable) {
        return context.get(variable);
    }

    public void updateContext(final Map<String, String> ctx) {
        for (final Entry<String, String> entry : ctx.entrySet())
            if (entry.getValue() == null || entry.getValue().equals("none")) {
                context.remove(entry.getKey());
            } else if (entry.getValue().startsWith("+") || entry.getValue().startsWith("-")) {
                // increment/decrement operators..
                if (!context.containsKey(entry.getKey())) {
                    // the numeric variable does not exist yet..
                    final String value = Double.toString(Double.parseDouble(entry.getValue()));
                    context.put(entry.getKey(), value);
                } else {
                    // the numeric variable already exists..
                    final String value = Double.toString(
                            Double.parseDouble(context.get(entry.getKey())) + Double.parseDouble(entry.getValue()));
                    context.put(entry.getKey(), value);
                }
            } else if (entry.getValue().startsWith("*")) {
                // increment/decrement operators..
                if (!context.containsKey(entry.getKey())) {
                    // the numeric variable does not exist yet..
                    context.put(entry.getKey(), "0");
                } else {
                    // the numeric variable already exists..
                    final String value = Double.toString(
                            Double.parseDouble(context.get(entry.getKey())) * Double.parseDouble(entry.getValue()));
                    context.put(entry.getKey(), value);
                }
            } else if (entry.getValue().startsWith("/")) {
                // increment/decrement operators..
                if (!context.containsKey(entry.getKey())) {
                    // the numeric variable does not exist yet..
                    context.put(entry.getKey(), "0");
                } else {
                    // the numeric variable already exists..
                    final String value = Double.toString(
                            Double.parseDouble(context.get(entry.getKey())) / Double.parseDouble(entry.getValue()));
                    context.put(entry.getKey(), value);
                }
            }
    }

    private void executeActions() {
        final List<Action> x_acts = ACTIONS.values().stream().filter(a -> a.getCondition().verify(this))
                .collect(Collectors.toList());
        if (x_acts.isEmpty()) {
            LOG.info("No action matches the current conditions: {}", context);
            executeAction(new Action("no_action", null, Arrays.asList("Non ho capito", "Scusa, puoi ripetere?"), null));
        } else {
            for (final Action action : x_acts) {
                try {
                    LOG.info("Current context is: {}", App.MAPPER.writeValueAsString(context));
                } catch (final JsonProcessingException e) {
                    LOG.error(e.getMessage(), e);
                }
                LOG.info("Selected action is: {}", action.getName());
                executeAction(action);
            }
        }
    }

    private void executeAction(Action action) {
        if (action.getContext() != null)
            // we update the context..
            updateContext(action.getContext());

        String response_text = action.getResponseTexts().get(random.nextInt(action.getResponseTexts().size()));
        try {
            App.MQTT_CLIENT.publish(prefix + "/system_utterance",
                    App.MAPPER.writeValueAsString(new StringWrapper(response_text)).getBytes(), App.QoS, false);
        } catch (MqttException | JsonProcessingException e) {
            LOG.error(e.getMessage(), e);
        }

        if (context.containsKey("success")) {
            if (Boolean.parseBoolean(context.get("success"))) {
                try {
                    App.MQTT_CLIENT.publish(prefix + "/done",
                            App.MAPPER
                                    .writeValueAsString(
                                            new LongArray(new long[] { Long.parseLong(context.get("sigma")) }))
                                    .getBytes(),
                            App.QoS, false);
                } catch (MqttException | JsonProcessingException e) {
                    LOG.error(e.getMessage(), e);
                }
            } else {
                try {
                    App.MQTT_CLIENT.publish(prefix + "/failure",
                            App.MAPPER
                                    .writeValueAsString(
                                            new LongArray(new long[] { Long.parseLong(context.get("sigma")) }))
                                    .getBytes(),
                            App.QoS, false);
                } catch (MqttException | JsonProcessingException e) {
                    LOG.error(e.getMessage(), e);
                }
            }
            context.remove("Command");
            context.remove("success");
        }
    }

    @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
    @JsonSubTypes({ @Type(value = CognitiveExercize.class, name = "CognitiveExercize"),
            @Type(value = Reminder.class, name = "Reminder"),
            @Type(value = PhysiologicalMeasurement.class, name = "PhysiologicalMeasurement") })
    private abstract static class Command {
    }

    @SuppressWarnings("unused")
    private static class CognitiveExercizes {
        private Collection<CognitiveExercize> atoms;
    }

    @SuppressWarnings("unused")
    private static class CognitiveExercize extends Command {

        private long sigma;
        private int ex_id;
    }

    @SuppressWarnings("unused")
    private static class Reminders {
        private Collection<Reminder> atoms;
    }

    @SuppressWarnings("unused")
    private static class Reminder extends Command {

        private long sigma;
        private int rem_id;
    }

    @SuppressWarnings("unused")
    private static class PhysiologicalMeasurements {
        private Collection<PhysiologicalMeasurement> atoms;
    }

    @SuppressWarnings("unused")
    private static class PhysiologicalMeasurement extends Command {

        private long sigma;
        private int pm_id;
    }
}
