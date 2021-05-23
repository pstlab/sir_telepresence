package it.cnr.istc.pst.sirobotics.telepresence.api;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class RobotConf {

    private final String[] starting, ending;

    @JsonCreator
    public RobotConf(@JsonProperty("starting") final String[] starting, @JsonProperty("ending") final String[] ending) {
        this.starting = starting;
        this.ending = ending;
    }

    public String[] getStarting() {
        return starting;
    }

    public String[] getEnding() {
        return ending;
    }
}
