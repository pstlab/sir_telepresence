package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.Map;
import java.util.stream.Collectors;

import javax.ws.rs.client.Client;
import javax.ws.rs.client.ClientBuilder;
import javax.ws.rs.client.Entity;
import javax.ws.rs.client.WebTarget;
import javax.ws.rs.core.MediaType;
import javax.ws.rs.core.Response;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class RasaClient {

    private static final Logger LOG = LoggerFactory.getLogger(RasaClient.class);
    private final Client client = ClientBuilder.newClient();
    private final WebTarget target;

    public RasaClient() {
        final String server_uri = "http://" + App.PROPERTIES.getProperty("rasa_host", "localhost") + ':'
                + App.PROPERTIES.getProperty("rasa_port", "5005");
        target = client.target(server_uri);
    }

    public JsonNode version() {
        final Response response = target.path("version").request(MediaType.APPLICATION_JSON).get();
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    public JsonNode utter(final String sender, final String message) {
        LOG.info("{} is saying {}", sender, message);
        final Response response = target.path("webhooks/rest/webhook").request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(new RasaMessage(sender, message), MediaType.APPLICATION_JSON));
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    public JsonNode get(final String sender) {
        LOG.info("{} is getting the state", sender);
        final Response response = target.path("conversations").path(sender).path("tracker")
                .queryParam("include_events", "NONE").request().get();
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    public JsonNode set(final String sender, final String slot, final String value) {
        LOG.info("{} is setting {} to {}", new Object[] { sender, slot, value });
        final Response response = target.path("conversations").path(sender).path("tracker/events")
                .request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(new SlotSet(slot, value), MediaType.APPLICATION_JSON));
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    public JsonNode trigger(final String sender, final String intent, final Map<String, Object> entities) {
        LOG.info("{} is triggering {} with entities {}", new Object[] { sender, intent, entities });
        final Response response = target.path("conversations").path(sender).path("trigger_intent")
                .queryParam("include_events", "NONE").request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(new IntentTrigger(intent, entities), MediaType.APPLICATION_JSON));
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    public JsonNode set(final String sender, final Map<String, String> slots) {
        LOG.info("{} is setting {}", sender, slots);
        final Response response = target
                .path("conversations").path(sender).path(
                        "tracker/events")
                .queryParam("include_events", "NONE").request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(slots.entrySet().stream().map(slot -> new SlotSet(slot.getKey(), slot.getValue()))
                        .collect(Collectors.toList()), MediaType.APPLICATION_JSON));
        try {
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (final JsonProcessingException e) {
            return null;
        }
    }

    @SuppressWarnings("unused")
    private static class RasaMessage {

        final String sender;
        final String message;

        public RasaMessage(final String sender, final String message) {
            this.sender = sender;
            this.message = message;
        }

        public String getSender() {
            return sender;
        }

        public String getMessage() {
            return message;
        }
    }

    @SuppressWarnings("unused")
    private static class SlotSet {

        final String event = "slot";
        final String name;
        final String value;
        final long timestamp = System.currentTimeMillis();

        public SlotSet(final String name, final String value) {
            this.name = name;
            this.value = value;
        }

        public String getEvent() {
            return event;
        }

        public String getName() {
            return name;
        }

        public String getValue() {
            return value;
        }

        public long getTimestamp() {
            return timestamp;
        }
    }

    @SuppressWarnings("unused")
    private static class IntentTrigger {

        final String name;
        final Map<String, Object> entities;

        public IntentTrigger(final String name, final Map<String, Object> entities) {
            this.name = name;
            this.entities = entities;
        }

        public String getName() {
            return name;
        }

        public Map<String, Object> getEntities() {
            return entities;
        }
    }
}
