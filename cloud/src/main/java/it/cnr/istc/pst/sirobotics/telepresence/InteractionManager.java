package it.cnr.istc.pst.sirobotics.telepresence;

import java.io.IOException;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

import com.fasterxml.jackson.databind.JsonNode;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.sirobotics.telepresence.api.StringWrapper;

public class InteractionManager {

    private static final Logger LOG = LoggerFactory.getLogger(InteractionManager.class);
    private static final Executor COMMAND_EXECUTOR = Executors.newSingleThreadExecutor();

    public InteractionManager(final String prefix) {
        JsonNode set = App.NLU_CLIENT.set(prefix, "mqtt_prefix", prefix);
        LOG.info(set.get("slots").toString());
        try {
            App.MQTT_CLIENT.subscribe(prefix + "/user_utterance", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    try {
                        final StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                                StringWrapper.class);
                        LOG.info("user says to " + prefix + ": " + string_message.data);
                        final JsonNode response = App.NLU_CLIENT.utter(prefix, string_message.data);
                        final String response_text = response.get("text").asText();
                        LOG.info("system " + prefix + " answers: " + string_message.data);
                        App.MQTT_CLIENT.publish(prefix + "/system_utterance",
                                App.MAPPER.writeValueAsString(new StringWrapper(response_text)).getBytes(), App.QoS,
                                false);
                    } catch (IOException | MqttException e) {
                        LOG.error("Cannot process the input text", e);
                    }
                });
            });
        } catch (MqttException e) {
            LOG.error("Cannot subscribe the interaction manager #" + prefix + " to the MQTT broker..", e);
        }
    }
}
