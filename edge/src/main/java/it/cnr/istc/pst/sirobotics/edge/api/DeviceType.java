package it.cnr.istc.pst.sirobotics.edge.api;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
@JsonSubTypes({ @Type(value = DeviceType.RobotType.class, name = "robot"),
        @Type(value = DeviceType.SensorType.class, name = "sensor") })
public abstract class DeviceType {

    private final long id;
    private final String name, description;

    @JsonCreator
    public DeviceType(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
            @JsonProperty("description") final String description) {
        this.id = id;
        this.name = name;
        this.description = description;
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

    public static class SensorType extends DeviceType {

        public SensorType(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
                @JsonProperty("description") final String description) {
            super(id, name, description);
        }
    }

    public static class RobotType extends DeviceType {

        public RobotType(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
                @JsonProperty("description") final String description) {
            super(id, name, description);
        }
    }
}
