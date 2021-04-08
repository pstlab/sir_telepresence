package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collection;
import java.util.Collections;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class House {

    private final long id;
    private final String name, description;
    private final Collection<Device> devices;

    @JsonCreator
    public House(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
            @JsonProperty("description") final String description,
            @JsonProperty("devices") final Collection<Device> devices) {
        this.id = id;
        this.name = name;
        this.description = description;
        this.devices = devices;
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

    public Collection<Device> getDevices() {
        return Collections.unmodifiableCollection(devices);
    }
}
