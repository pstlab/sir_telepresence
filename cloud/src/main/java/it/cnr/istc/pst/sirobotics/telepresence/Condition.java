package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.Collection;
import java.util.Collections;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonSubTypes.Type;
import com.fasterxml.jackson.annotation.JsonTypeInfo;

@JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "type")
@JsonSubTypes({ @Type(value = Condition.AndCondition.class, name = "and"),
        @Type(value = Condition.OrCondition.class, name = "or"),
        @Type(value = Condition.VarCondition.class, name = "var"),
        @Type(value = Condition.NumVarCondition.class, name = "num-var") })
public abstract class Condition {

    public static final String ANY = "any";
    public static final String NONE = "none";

    public abstract boolean verify(InteractionManager ctx);

    public static class AndCondition extends Condition {

        private final Collection<Condition> conditions;

        public AndCondition(@JsonProperty("conditions") final Collection<Condition> conditions) {
            this.conditions = conditions;
        }

        /**
         * @return the conditions
         */
        public Collection<Condition> getConditions() {
            return Collections.unmodifiableCollection(conditions);
        }

        @Override
        public boolean verify(final InteractionManager ctx) {
            return conditions.stream().allMatch(cond -> cond.verify(ctx));
        }
    }

    public static class OrCondition extends Condition {

        private final Collection<Condition> conditions;

        public OrCondition(@JsonProperty("conditions") final Collection<Condition> conditions) {
            this.conditions = conditions;
        }

        /**
         * @return the conditions
         */
        public Collection<Condition> getConditions() {
            return Collections.unmodifiableCollection(conditions);
        }

        @Override
        public boolean verify(final InteractionManager ctx) {
            return conditions.stream().anyMatch(cond -> cond.verify(ctx));
        }
    }

    public static class VarCondition extends Condition {

        private final String variable, value;
        private final VarCondType cond_type;

        public VarCondition(@JsonProperty("variable") final String variable, @JsonProperty("value") final String value,
                @JsonProperty("cond_type") final VarCondType cond_type) {
            this.variable = variable;
            this.cond_type = cond_type == null ? VarCondType.Eq : cond_type;
            this.value = value;
        }

        /**
         * @return the variable
         */
        public String getVariable() {
            return variable;
        }

        /**
         * @return the value
         */
        public String getValue() {
            return value;
        }

        /**
         * @return the var_cond_type
         */
        @JsonProperty("cond_type")
        public VarCondType getCondType() {
            return cond_type;
        }

        @Override
        public boolean verify(final InteractionManager ctx) {
            switch (value) {
                case ANY:
                    switch (cond_type) {
                        case Eq:
                            return ctx.hasContextVariable(variable);
                        case NEq:
                            return !ctx.hasContextVariable(variable);
                        default:
                            throw new IllegalArgumentException(String.valueOf(cond_type));
                    }
                case NONE:
                    switch (cond_type) {
                        case Eq:
                            return !ctx.hasContextVariable(variable);
                        case NEq:
                            return ctx.hasContextVariable(variable);
                        default:
                            throw new IllegalArgumentException(String.valueOf(cond_type));
                    }
                default: {
                    final String ctx_value = ctx.getContextValue(variable);
                    switch (cond_type) {
                        case Eq:
                            return ctx_value != null && ctx_value.equals(value);
                        case NEq:
                            return ctx_value != null && !ctx_value.equals(value);
                        default:
                            throw new IllegalArgumentException(String.valueOf(cond_type));
                    }
                }
            }
        }

        public enum VarCondType {
            Eq, NEq
        }
    }

    public static class NumVarCondition extends Condition {

        private final String variable;
        private final double value;
        private final NumVarCondType cond_type;

        public NumVarCondition(@JsonProperty("variable") final String variable,
                @JsonProperty("value") final double value, @JsonProperty("cond_type") final NumVarCondType cond_type) {
            this.variable = variable;
            this.cond_type = cond_type == null ? NumVarCondType.Eq : cond_type;
            this.value = value;
        }

        /**
         * @return the variable
         */
        public String getVariable() {
            return variable;
        }

        /**
         * @return the value
         */
        public double getValue() {
            return value;
        }

        /**
         * @return the var_cond_type
         */
        @JsonProperty("cond_type")
        public NumVarCondType getCondType() {
            return cond_type;
        }

        @Override
        public boolean verify(final InteractionManager ctx) {
            final String ctx_value = ctx.getContextValue(variable);
            switch (cond_type) {
                case Eq:
                    return ctx_value != null && Double.parseDouble(ctx_value) == value;
                case GEq:
                    return ctx_value != null && Double.parseDouble(ctx_value) >= value;
                case Gt:
                    return ctx_value != null && Double.parseDouble(ctx_value) > value;
                case LEq:
                    return ctx_value != null && Double.parseDouble(ctx_value) <= value;
                case Lt:
                    return ctx_value != null && Double.parseDouble(ctx_value) < value;
                case NEq:
                    return ctx_value != null && Double.parseDouble(ctx_value) != value;
                default:
                    throw new IllegalArgumentException(String.valueOf(cond_type));
            }
        }

        public enum NumVarCondType {
            Lt, LEq, Eq, GEq, Gt, NEq
        }
    }
}