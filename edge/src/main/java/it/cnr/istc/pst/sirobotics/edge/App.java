package it.cnr.istc.pst.sirobotics.edge;

import org.eclipse.paho.client.mqttv3.IMqttClient;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.context.annotation.Bean;

@SpringBootApplication
public class App {

    public static void main(String[] args) {
        SpringApplication.run(App.class, args);
    }

    @Bean
    @ConfigurationProperties(prefix = "mqtt")
    public MqttConnectOptions mqttConnectOptions() {
        return new MqttConnectOptions();
    }

    @Bean
    public IMqttClient mqttClient(@Value("${mqtt.clientId}") String clientId,
            @Value("${mqtt.hostname}") String hostname, @Value("${mqtt.port}") int port) throws MqttException {
        IMqttClient mqtt_client = new MqttClient("tcp://" + hostname + ":" + port, clientId);
        mqtt_client.connect(mqttConnectOptions());
        return mqtt_client;
    }
}
