package it.cnr.istc.pst.sirobotics.telepresence.api;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class House {

    private final long id;
    private final String name, description;
    private final Device[] devices;
    private final User[] users;

    @JsonCreator
    public House(@JsonProperty("id") final long id, @JsonProperty("name") final String name,
            @JsonProperty("description") final String description, @JsonProperty("devices") final Device[] devices,
            @JsonProperty("users") final User[] users) {
        this.id = id;
        this.name = name;
        this.description = description;
        this.devices = devices;
        this.users = users;
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

    public Device[] getDevices() {
        return devices;
    }

    public User[] getUsers() {
        return users;
    }
}
