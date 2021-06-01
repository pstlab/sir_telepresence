package it.cnr.istc.pst.sirobotics.telepresence.api;

import java.util.Collections;
import java.util.HashSet;
import java.util.Set;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;

public class ExecConf {

    private final Set<String> notify_starting, notify_ending, auto_done;

    @JsonCreator
    public ExecConf(@JsonProperty("notify-start") final Set<String> notify_starting,
            @JsonProperty("notify-end") final Set<String> notify_ending,
            @JsonProperty("auto-done") final Set<String> auto_done) {
        this.notify_starting = notify_starting;
        this.notify_ending = notify_ending;
        this.auto_done = auto_done;
    }

    public Set<String> getRelevantPredicates() {
        Set<String> relevant_predicates = new HashSet<>();
        relevant_predicates.addAll(notify_starting);
        relevant_predicates.addAll(notify_ending);
        relevant_predicates.addAll(auto_done);
        return relevant_predicates;
    }

    public Set<String> getNotifyStarting() {
        if (notify_starting == null)
            return notify_starting;
        return Collections.unmodifiableSet(notify_starting);
    }

    public Set<String> getNotifyEnding() {
        if (notify_ending == null)
            return notify_ending;
        return Collections.unmodifiableSet(notify_ending);
    }

    public Set<String> getAutoDone() {
        if (auto_done == null)
            return auto_done;
        return Collections.unmodifiableSet(auto_done);
    }
}
