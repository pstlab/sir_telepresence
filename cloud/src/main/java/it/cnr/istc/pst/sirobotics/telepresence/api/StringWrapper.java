package it.cnr.istc.pst.sirobotics.telepresence.api;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class StringWrapper {

    public final String data;

    @JsonCreator
    public StringWrapper(@JsonProperty("data") final String data) {
        this.data = data;
    }
}
