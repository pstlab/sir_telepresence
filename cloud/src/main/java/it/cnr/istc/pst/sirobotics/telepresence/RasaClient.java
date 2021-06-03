package it.cnr.istc.pst.sirobotics.telepresence;

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
                + App.PROPERTIES.getProperty("rasa_port", "5005") + "/webhooks/rest/webhook";
        target = client.target(server_uri);
    }

    public JsonNode version() throws JsonProcessingException {
        final Response response = target.path("version").request(MediaType.APPLICATION_JSON).get();
        return App.MAPPER.readTree(response.readEntity(String.class));
    }

    public JsonNode utter(final String sender, final String message) throws JsonProcessingException {
        LOG.info("Parsing message: {}", message);
        final Response response = target.request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(new RasaMessage(sender, message), MediaType.APPLICATION_JSON));
        return App.MAPPER.readTree(response.readEntity(String.class));
    }

    @SuppressWarnings("unused")
    private static class RasaMessage {

        String sender;
        String message;

        public RasaMessage(final String sender, final String message) {
            this.sender = sender;
            this.message = message;
        }

        public String getText() {
            return message;
        }
    }
}
