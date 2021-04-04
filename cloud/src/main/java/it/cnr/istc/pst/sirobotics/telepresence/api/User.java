package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collection;
import java.util.Collections;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class User {

    private final long id;
    private final String email;
    private final String first_name;
    private final String last_name;
    private final Collection<String> roles;
    private final boolean online;

    @JsonCreator
    public User(@JsonProperty("id") final long id, @JsonProperty("email") final String email,
            @JsonProperty("first_name") final String first_name, @JsonProperty("last_name") final String last_name,
            @JsonProperty("roles") final Collection<String> roles, @JsonProperty("online") final boolean online) {
        this.id = id;
        this.email = email;
        this.first_name = first_name;
        this.last_name = last_name;
        this.roles = roles;
        this.online = online;
    }

    public long getId() {
        return id;
    }

    public String getEmail() {
        return email;
    }

    public String getFirstName() {
        return first_name;
    }

    public String getLastName() {
        return last_name;
    }

    public Collection<String> getRoles() {
        return Collections.unmodifiableCollection(roles);
    }

    public boolean isOnline() {
        return online;
    }
}
