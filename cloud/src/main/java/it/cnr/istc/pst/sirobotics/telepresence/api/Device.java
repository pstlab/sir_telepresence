package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collection;
import java.util.Collections;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
@JsonSubTypes({ @Type(value = Device.Robot.class, name = "robot"),
        @Type(value = Device.Sensor.class, name = "sensor") })
public class Device {

    private final long id;
    private final String name, description;
    private final DeviceType type;

    @JsonCreator
    public Device(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
            @JsonProperty("description") final String description, @JsonProperty("type") final DeviceType type) {
        this.id = id;
        this.name = name;
        this.description = description;
        this.type = type;
    }

    public long getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public String getDescription() {
        return description;
    }

    public DeviceType getType() {
        return type;
    }

    public static class Robot extends Device {

        private final Collection<Device> devices;

        @JsonCreator
        public Robot(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
                @JsonProperty("description") final String description, @JsonProperty("type") final DeviceType type,
                @JsonProperty("devices") final Collection<Device> devices) {
            super(id, name, description, type);
            this.devices = devices;
        }

        public Collection<Device> getDevices() {
            if (devices == null)
                return devices;
            return Collections.unmodifiableCollection(devices);
        }
    }

    public static class Sensor extends Device {

        private final Collection<Data> data;

        @JsonCreator
        public Sensor(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
                @JsonProperty("description") final String description, @JsonProperty("type") final DeviceType type,
                @JsonProperty("data") final Collection<Data> data) {
            super(id, name, description, type);
            this.data = data;
        }

        public Collection<Data> getData() {
            if (data == null)
                return data;
            return Collections.unmodifiableCollection(data);
        }

        public static class Data {

            private final long timestamp;
            private final String data;

            @JsonCreator
            public Data(@JsonProperty("timestamp") final long timestamp, @JsonProperty("data") final String data) {
                this.timestamp = timestamp;
                this.data = data;
            }

            public long getTimestamp() {
                return timestamp;
            }

            public String getData() {
                return data;
            }
        }
    }
}
