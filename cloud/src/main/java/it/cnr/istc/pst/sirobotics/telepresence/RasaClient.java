package it.cnr.istc.pst.sirobotics.telepresence;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
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
        target = client.target(App.PROPERTIES.getProperty("rasa_host"));
    }

    public JsonNode parse(String message) throws JsonProcessingException {
        Response response = target.path("model/parse").request(MediaType.APPLICATION_JSON)
                .post(Entity.entity(new RasaMessage(message), MediaType.APPLICATION_JSON));
        return App.MAPPER.readTree(response.readEntity(String.class));
    }

    public JsonNode train() {
        try {
            Response response = target.path("model/parse").request(MediaType.APPLICATION_JSON).post(Entity
                    .entity(new RasaModel(readFile("config.yml"), readFile("nlu.yml")), MediaType.APPLICATION_JSON));
            return App.MAPPER.readTree(response.readEntity(String.class));
        } catch (IOException ex) {
            LOG.error("Cannot read model", ex);
            return null;
        }
    }

    private String readFile(String file) throws IOException {
        try (InputStream is = RasaClient.class.getResourceAsStream(file)) {
            if (is == null)
                return null;
            try (InputStreamReader isr = new InputStreamReader(is); BufferedReader reader = new BufferedReader(isr)) {
                return reader.lines().collect(Collectors.joining(System.lineSeparator()));
            }
        }
    }

    @SuppressWarnings("unused")
    private static class RasaMessage {

        String text;

        public RasaMessage(String text) {
            this.text = text;
        }

        public String getText() {
            return text;
        }
    }

    @SuppressWarnings("unused")
    private static class RasaModel {

        String config;
        String nlu;

        public RasaModel(String config, String nlu) {
            this.config = config;
            this.nlu = nlu;
        }
    }
}
