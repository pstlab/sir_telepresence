package it.cnr.istc.pst.sirobotics.telepresence.api;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class LongArray {

    public final long[] data;

    @JsonCreator
    public LongArray(@JsonProperty("data") final long[] data) {
        this.data = data;
    }
}
