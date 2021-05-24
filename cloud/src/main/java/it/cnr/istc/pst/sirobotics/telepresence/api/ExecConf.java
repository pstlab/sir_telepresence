package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collections;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class ExecConf {

    private final Set<String> starting, done, ending;

    @JsonCreator
    public ExecConf(@JsonProperty("starting") final Set<String> starting, @JsonProperty("done") final Set<String> done,
            @JsonProperty("ending") final Set<String> ending) {
        this.starting = starting;
        this.done = done;
        this.ending = ending;
    }

    public Set<String> getStarting() {
        if (starting == null)
            return starting;
        return Collections.unmodifiableSet(starting);
    }

    public Set<String> getDone() {
        if (done == null)
            return done;
        return Collections.unmodifiableSet(done);
    }

    public Set<String> getEnding() {
        if (ending == null)
            return ending;
        return Collections.unmodifiableSet(ending);
    }
}
