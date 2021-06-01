package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Action {

    private final String name;
    private final Condition condition;
    private final List<String> response_text;
    private final Map<String, String> context;

    public Action(@JsonProperty("name") String name, @JsonProperty("condition") Condition condition,
            @JsonProperty("response_texts") List<String> response_text,
            @JsonProperty("context") Map<String, String> context) {
        this.name = name;
        this.condition = condition;
        this.response_text = response_text;
        this.context = context;
    }

    /**
     * @return the name
     */
    public String getName() {
        return name;
    }

    /**
     * @return the condition
     */
    public Condition getCondition() {
        return condition;
    }

    /**
     * @return the response_text
     */
    @JsonProperty("response_texts")
    public List<String> getResponseTexts() {
        return Collections.unmodifiableList(response_text);
    }

    /**
     * @return the context
     */
    public Map<String, String> getContext() {
        return context != null ? Collections.unmodifiableMap(context) : null;
    }
}
