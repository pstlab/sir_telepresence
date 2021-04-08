package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collection;
import java.util.Collections;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class House {

    private final long id;
    private final Collection<Device> devices;

    @JsonCreator
    public House(@JsonProperty("id") final long id, @JsonProperty("devices") final Collection<Device> devices) {
        this.id = id;
        this.devices = devices;
    }

    public long getId() {
        return id;
    }

    public Collection<Device> getDevices() {
        return Collections.unmodifiableCollection(devices);
    }
}
