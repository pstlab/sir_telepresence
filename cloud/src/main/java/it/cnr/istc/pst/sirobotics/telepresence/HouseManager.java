package it.cnr.istc.pst.sirobotics.telepresence;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.persistence.EntityManager;

import com.fasterxml.jackson.annotation.JsonAutoDetect;
import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonSubTypes;
import com.fasterxml.jackson.annotation.JsonTypeInfo;
import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Predicate;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.SolverException;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.Type;
import it.cnr.istc.pst.oratio.timelines.ExecutorException;
import it.cnr.istc.pst.oratio.timelines.ExecutorListener;
import it.cnr.istc.pst.oratio.timelines.PropositionalAgent;
import it.cnr.istc.pst.oratio.timelines.ReusableResource;
import it.cnr.istc.pst.oratio.timelines.StateVariable;
import it.cnr.istc.pst.oratio.timelines.Timeline;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.oratio.timelines.TimelinesList;
import it.cnr.istc.pst.sirobotics.telepresence.api.RobotConf;
import it.cnr.istc.pst.sirobotics.telepresence.db.DeviceEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorDataEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class HouseManager {

    private static final Logger LOG = LoggerFactory.getLogger(HouseManager.class);
    private static final ScheduledExecutorService EXECUTOR = Executors
            .newScheduledThreadPool(Runtime.getRuntime().availableProcessors());
    private final HouseEntity house;
    private final Map<String, SolverManager> solver_managers = new HashMap<>();

    public HouseManager(final HouseEntity house) {
        this.house = house;
        final String prefix = Long.toString(house.getId());
        solver_managers.put(prefix, new SolverManager(prefix, "{}"));

        for (final DeviceEntity device_entity : house.getDevices()) {
            if (device_entity instanceof RobotEntity) {
                addRobot((RobotEntity) device_entity);
            } else if (device_entity instanceof SensorEntity) {
                addSensor((SensorEntity) device_entity);
            }
        }
    }

    public void addRobot(final RobotEntity robot_entity) {
        final String prefix = house.getId() + "/" + robot_entity.getId();
        solver_managers.put(prefix,
                new SolverManager(prefix, ((RobotTypeEntity) robot_entity.getType()).getConfiguration()));
    }

    public void addSensor(final SensorEntity sensor_entity) {
        try {
            App.MQTT_CLIENT.subscribe(house.getId() + "/" + sensor_entity.getId(), (topic, message) -> {
                final EntityManager em = App.EMF.createEntityManager();
                final SensorDataEntity data = new SensorDataEntity();
                data.setSensingTime(new Date());
                data.setRawData(new String(message.getPayload()));
                em.getTransaction().begin();
                ((SensorEntity) sensor_entity).addData(data);
                em.persist(data);
                em.getTransaction().commit();
                em.close();
            });
        } catch (final MqttException ex) {
            LOG.error("Cannot subscribe the device #" + sensor_entity.getId() + " to the MQTT broker..", ex);
        }
    }

    static class StringWrapper {

        final String data;

        @JsonCreator
        StringWrapper(@JsonProperty("data") final String data) {
            this.data = data;
        }
    }

    static class SolverManager implements StateListener, GraphListener, ExecutorListener {

        private final String prefix;
        private Solver solver = null;
        private TimelinesList timelines = null;
        private final Map<Long, Atom> c_atoms = new HashMap<>();
        private TimelinesExecutor tl_exec = null;
        private ScheduledFuture<?> scheduled_feature;
        private final Map<Long, Flaw> flaws = new HashMap<>();
        private Flaw current_flaw = null;
        private final Map<Long, Resolver> resolvers = new HashMap<>();
        private Resolver current_resolver = null;
        private Rational current_time = new Rational();
        private final Set<String> disp_s_preds = new HashSet<>();
        private final Set<String> disp_e_preds = new HashSet<>();
        private SolverState state = null;

        public SolverManager(final String prefix, final String configuration) {
            this.prefix = prefix;

            try {
                RobotConf conf = App.MAPPER.readValue(configuration, RobotConf.class);
                if (conf.getStarting() != null)
                    Arrays.stream(conf.getStarting()).forEach(pred -> disp_s_preds.add(pred));
                if (conf.getEnding() != null)
                    Arrays.stream(conf.getEnding()).forEach(pred -> disp_e_preds.add(pred));
            } catch (JsonProcessingException ex) {
                LOG.error("Cannot read config file..", ex);
            }

            try {
                App.MQTT_CLIENT.subscribe(prefix + "/plan", (topic, message) -> {
                    if (state != SolverState.Waiting) {
                        LOG.info("[" + prefix + "] Solver state is " + state.name()
                                + " and cannot read a new problem..");
                        return;
                    }

                    setState(SolverState.Solving);

                    // we read the problem..
                    LOG.info("[" + prefix + "] Reading the problem..");
                    final StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                            StringWrapper.class);
                    try {
                        solver.read(string_message.data);
                    } catch (final SolverException e) {
                        LOG.error("Cannot read the given problem", e);
                    }

                    // we solve the problem..
                    LOG.info("[" + prefix + "] Solving the problem..");
                    try {
                        solver.solve();
                    } catch (final SolverException e) {
                        LOG.error("Cannot solve the given problem", e);
                    }
                });

                App.MQTT_CLIENT.subscribe(prefix + "/done", (topic, message) -> {
                    scheduled_feature.cancel(false);
                    final LongArray long_array_message = App.MAPPER.readValue(message.getPayload(), LongArray.class);
                    tl_exec.done(long_array_message.data);
                });

                App.MQTT_CLIENT.subscribe(prefix + "/failure", (topic, message) -> {
                    scheduled_feature.cancel(false);
                    final LongArray long_array_message = App.MAPPER.readValue(message.getPayload(), LongArray.class);
                    tl_exec.failure(long_array_message.data);
                });

                App.MQTT_CLIENT.subscribe(prefix + "/nlp/in", (topic, message) -> {
                    final StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                            StringWrapper.class);
                    final JsonNode parse = App.NLU_CLIENT.parse(string_message.data);
                    App.MQTT_CLIENT.publish(prefix + "/nlp/intent",
                            App.MAPPER.writeValueAsString(new StringWrapper(parse.get("intent").toString())).getBytes(),
                            App.QoS, true);
                });
            } catch (final MqttException ex) {
                LOG.error("Cannot subscribe the house #" + prefix + " to the MQTT broker..", ex);
            }

            reset();
        }

        public SolverState getState() {
            return state;
        }

        public void setState(final SolverState state) {
            this.state = state;
            fireStateChanged();
        }

        public void reset() {
            LOG.info("[" + prefix + "] Resetting the solver..");
            solver = new Solver();
            solver.addStateListener(this);
            timelines = new TimelinesList(solver);
            tl_exec = new TimelinesExecutor(solver, new Rational(1));
            tl_exec.addExecutorListener(this);
            setState(SolverState.Waiting);
        }

        Collection<Object> getTimelines() {
            final Collection<Object> c_tls = new ArrayList<>();
            for (final Timeline<?> tl : timelines) {
                if (tl instanceof StateVariable) {
                    c_tls.add(toTimeline((StateVariable) tl));
                } else if (tl instanceof ReusableResource) {
                    c_tls.add(toTimeline((ReusableResource) tl));
                } else if (tl instanceof PropositionalAgent) {
                    c_tls.add(toTimeline((PropositionalAgent) tl));
                }
            }
            return c_tls;
        }

        private static SVTimeline toTimeline(final StateVariable sv) {
            return new SVTimeline(sv.getName(), sv.getOrigin().doubleValue(), sv.getHorizon().doubleValue(), sv
                    .getValues().stream()
                    .map(val -> new SVTimeline.Value(
                            val.getAtoms().stream().map(atm -> atm.toString()).collect(Collectors.joining(", ")),
                            val.getFrom().doubleValue(), val.getTo().doubleValue(),
                            val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                    .collect(Collectors.toList()));
        }

        private static RRTimeline toTimeline(final ReusableResource rr) {
            return new RRTimeline(rr.getName(), rr.getCapacity().doubleValue(), rr.getOrigin().doubleValue(),
                    rr.getHorizon().doubleValue(),
                    rr.getValues().stream()
                            .map(val -> new RRTimeline.Value(val.getUsage().doubleValue(), val.getFrom().doubleValue(),
                                    val.getTo().doubleValue(),
                                    val.getAtoms().stream().map(atm -> atm.getSigma()).collect(Collectors.toList())))
                            .collect(Collectors.toList()));
        }

        private static Agent toTimeline(final PropositionalAgent pa) {
            return new Agent(pa.getName(), pa.getOrigin().doubleValue(), pa.getHorizon().doubleValue(),
                    pa.getValues().stream().map(val -> new Agent.Value(val.getAtom().toString(),
                            val.getFrom().doubleValue(), val.getTo().doubleValue(), val.getAtom().getSigma()))
                            .collect(Collectors.toList()));
        }

        @Override
        public void log(final String log) {
            LOG.info("[" + prefix + "] " + log);
        }

        @Override
        public void read(final String script) {
        }

        @Override
        public void read(final String[] files) {
        }

        @Override
        public void stateChanged() {
        }

        @Override
        public void startedSolving() {
            LOG.info("[" + prefix + "] Solving the problem..");
            setState(SolverState.Solving);
        }

        @Override
        public void solutionFound() {
            if (state == SolverState.Solving) {
                LOG.info("[" + prefix + "] Solution found..");
                setState(SolverState.Solution);

                c_atoms.clear();

                for (final Type t : solver.getTypes().values())
                    for (final Predicate p : t.getPredicates().values())
                        p.getInstances().stream().map(atm -> (Atom) atm)
                                .filter(atm -> (atm.getState() == Atom.AtomState.Active))
                                .forEach(atm -> c_atoms.put(atm.getSigma(), atm));

                // we execute the solution..
                LOG.info("[" + prefix + "] Starting plan execution..");
                setState(SolverState.Executing);
                scheduled_feature = EXECUTOR.scheduleAtFixedRate(() -> {
                    try {
                        tl_exec.tick();
                    } catch (final ExecutorException e) {
                        LOG.error("Cannot execute the solution..", e);
                        scheduled_feature.cancel(false);
                        reset();
                    }
                }, 0, 1, TimeUnit.SECONDS);
            } else
                LOG.info("[" + prefix + "] Solution updated..");

            if (current_flaw != null)
                current_flaw.current = false;
            current_flaw = null;
            if (current_resolver != null)
                current_resolver.current = false;
            current_resolver = null;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin")) {
                        UserController.ONLINE.get(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new Message.Timelines(prefix, getTimelines())));
                        UserController.ONLINE.get(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new Message.SolutionFound(prefix)));
                        UserController.ONLINE.get(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new Message.Tick(prefix, current_time)));
                    }
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        }

        @Override
        public void inconsistentProblem() {
            LOG.info("[" + prefix + "] Inconsistent problem..");
            setState(SolverState.Inconsistent);
            reset();
        }

        @Override
        public void flawCreated(final long id, final long[] causes, final String label, final State state,
                final Bound position) {
            final Flaw c_flaw = new Flaw(id,
                    Arrays.stream(causes).mapToObj(r_id -> resolvers.get(r_id)).toArray(Resolver[]::new), label, state,
                    position);
            Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
            flaws.put(id, c_flaw);
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin"))
                        UserController.ONLINE.get(ue.getId()).send(App.MAPPER.writeValueAsString(
                                new Message.FlawCreated(prefix, id, causes, label, (byte) state.ordinal(), position)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        }

        @Override
        public void flawStateChanged(final long id, final State state) {
        }

        @Override
        public void flawCostChanged(final long id, final Rational cost) {
        }

        @Override
        public void flawPositionChanged(final long id, final Bound position) {
        }

        @Override
        public void currentFlaw(final long id) {
        }

        @Override
        public void resolverCreated(final long id, final long effect, final Rational cost, final String label,
                final State state) {
        }

        @Override
        public void resolverStateChanged(final long id, final State state) {
        }

        @Override
        public void currentResolver(final long id) {
        }

        @Override
        public void causalLinkAdded(final long flaw, final long resolver) {
        }

        @Override
        public void tick(final Rational current_time) {
            LOG.info("[" + prefix + "] Current time: " + current_time.toString());
            this.current_time = current_time;
            try {
                if (((ArithItem) solver.get("horizon")).getValue().leq(current_time)) {
                    LOG.info("Nothing more to execute..");
                    scheduled_feature.cancel(false);
                    reset();
                }
            } catch (final NoSuchFieldException e) {
                e.printStackTrace();
            }
        }

        @Override
        public void startingAtoms(final long[] atoms) {
            LOG.info("[" + prefix + "] Starting atoms: " + Arrays.toString(atoms));
            final Map<Type, Collection<Atom>> starting_atoms = new IdentityHashMap<>();
            for (int i = 0; i < atoms.length; i++) {
                final Atom starting_atom = c_atoms.get(atoms[i]);
                if (disp_s_preds.contains(starting_atom.getType().getName())) {
                    Collection<Atom> c_atms = starting_atoms.get(starting_atom.getType());
                    if (c_atms == null) {
                        c_atms = new ArrayList<>();
                        starting_atoms.put(starting_atom.getType(), c_atms);
                    }
                    c_atms.add(starting_atom);
                }
            }
            try {
                for (final Entry<Type, Collection<Atom>> entry : starting_atoms.entrySet())
                    App.MQTT_CLIENT.publish(prefix + "/" + entry.getKey().getName(),
                            App.MAPPER.writeValueAsString(new Command(entry.getValue().stream()
                                    .map(atm -> new CommandAtom(atm, true, false)).collect(Collectors.toList())))
                                    .getBytes(),
                            App.QoS, false);
            } catch (final JsonProcessingException | MqttException ex) {
                LOG.error("Cannot create MQTT message..", ex);
            }
        }

        @Override
        public void endingAtoms(final long[] atoms) {
            LOG.info("[" + prefix + "] Ending atoms: {}" + Arrays.toString(atoms));
            final Map<Type, Collection<Atom>> ending_atoms = new IdentityHashMap<>();
            for (int i = 0; i < atoms.length; i++) {
                final Atom ending_atom = c_atoms.get(atoms[i]);
                if (disp_e_preds.contains(ending_atom.getType().getName())) {
                    Collection<Atom> c_atms = ending_atoms.get(ending_atom.getType());
                    if (c_atms == null) {
                        c_atms = new ArrayList<>();
                        ending_atoms.put(ending_atom.getType(), c_atms);
                    }
                    c_atms.add(ending_atom);
                }
            }
            try {
                for (final Entry<Type, Collection<Atom>> entry : ending_atoms.entrySet())
                    App.MQTT_CLIENT.publish(prefix + "/" + entry.getKey().getName(),
                            App.MAPPER.writeValueAsString(new Command(entry.getValue().stream()
                                    .map(atm -> new CommandAtom(atm, false, true)).collect(Collectors.toList())))
                                    .getBytes(),
                            App.QoS, false);
            } catch (final JsonProcessingException | MqttException ex) {
                LOG.error("Cannot create MQTT message..", ex);
            }
        }

        private void fireStateChanged() {
            try {
                App.MQTT_CLIENT.publish(prefix + "/planner",
                        App.MAPPER.writeValueAsString(new StringWrapper(state.name())).getBytes(), App.QoS, true);
            } catch (final MqttException | JsonProcessingException ex) {
                LOG.error("Cannot create MQTT message..", ex);
            }
        }

        public enum SolverState {
            Waiting, Solving, Solution, Inconsistent, Executing;
        }

        static class LongArray {

            final long[] data;

            @JsonCreator
            LongArray(@JsonProperty("data") final long[] data) {
                this.data = data;
            }
        }

        @SuppressWarnings("unused")
        private static final class Command {

            private final Collection<CommandAtom> atoms;

            private Command(final Collection<CommandAtom> atoms) {
                this.atoms = atoms;
            }
        }

        @SuppressWarnings("unused")
        static final class CommandAtom {

            private final Atom atom;
            private final boolean starting, ending;

            private CommandAtom(final Atom atom, final boolean starting, final boolean ending) {
                this.atom = atom;
                this.starting = starting;
                this.ending = ending;
            }

            public Atom getAtom() {
                return atom;
            }

            public boolean isStarting() {
                return starting;
            }

            public boolean isEnding() {
                return ending;
            }
        }

        @SuppressWarnings("unused")
        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class SVTimeline {

            private final String type = "state-variable";
            private final String name;
            private final double origin, horizon;
            private final List<Value> values;

            private SVTimeline(final String name, final double origin, final double horizon, final List<Value> values) {
                this.name = name;
                this.origin = origin;
                this.horizon = horizon;
                this.values = values;
            }

            @JsonAutoDetect(fieldVisibility = Visibility.ANY)
            private static class Value {

                private final String name;
                private final double from, to;
                private final Collection<Long> atoms;

                private Value(final String name, final double from, final double to, final Collection<Long> atoms) {
                    this.name = name;
                    this.from = from;
                    this.to = to;
                    this.atoms = atoms;
                }
            }
        }

        @SuppressWarnings("unused")
        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class RRTimeline {

            private final String type = "reusable-resource";
            private final String name;
            private final double capacity;
            private final double origin, horizon;
            private final List<Value> values;

            private RRTimeline(final String name, final double capacity, final double origin, final double horizon,
                    final List<Value> values) {
                this.name = name;
                this.capacity = capacity;
                this.origin = origin;
                this.horizon = horizon;
                this.values = values;
            }

            @JsonAutoDetect(fieldVisibility = Visibility.ANY)
            private static class Value {

                private final double usage;
                private final double from, to;
                private final Collection<Long> atoms;

                private Value(final double usage, final double from, final double to, final Collection<Long> atoms) {
                    this.usage = usage;
                    this.from = from;
                    this.to = to;
                    this.atoms = atoms;
                }
            }
        }

        @SuppressWarnings("unused")
        @JsonAutoDetect(fieldVisibility = Visibility.ANY)
        private static class Agent {

            private final String type = "agent";
            private final String name;
            private final double origin, horizon;
            private final List<Value> values;

            private Agent(final String name, final double origin, final double horizon, final List<Value> values) {
                this.name = name;
                this.origin = origin;
                this.horizon = horizon;
                this.values = values;
            }

            @JsonAutoDetect(fieldVisibility = Visibility.ANY)
            private static class Value {

                private final String name;
                private final double from, to;
                private final Long atom;

                private Value(final String name, final double from, final double to, final Long atom) {
                    this.name = name;
                    this.from = from;
                    this.to = to;
                    this.atom = atom;
                }
            }
        }

        @JsonSerialize(using = FlawSerializer.class)
        class Flaw {

            private final long id;
            private final Resolver[] causes;
            private final String label;
            private final State state;
            private final Bound position;
            private final Rational cost = Rational.POSITIVE_INFINITY;
            private boolean current = false;

            private Flaw(final long id, final Resolver[] causes, final String label, final State state,
                    final Bound position) {
                this.id = id;
                this.causes = causes;
                this.label = label;
                this.state = state;
                this.position = position;
            }
        }

        private static class FlawSerializer extends StdSerializer<Flaw> {

            private static final long serialVersionUID = 1L;

            private FlawSerializer() {
                super(Flaw.class);
            }

            @Override
            public void serialize(final Flaw flaw, final JsonGenerator gen, final SerializerProvider provider)
                    throws IOException {
                gen.writeStartObject();
                gen.writeNumberField("id", flaw.id);
                gen.writeArrayFieldStart("causes");
                Arrays.stream(flaw.causes).forEach(c -> {
                    try {
                        gen.writeNumber(c.id);
                    } catch (final IOException e) {
                        LOG.error("Cannot serialize", e);
                    }
                });
                gen.writeEndArray();
                gen.writeStringField("label", flaw.label);
                gen.writeNumberField("state", flaw.state.ordinal());

                if (flaw.position.min != -Bound.INF || flaw.position.max != Bound.INF)
                    gen.writeObjectField("position", flaw.position);

                gen.writeObjectField("cost", flaw.cost);

                gen.writeBooleanField("current", flaw.current);

                gen.writeEndObject();
            }
        }

        @JsonSerialize(using = ResolverSerializer.class)
        class Resolver {

            private final long id;
            private final Flaw effect;
            private final String label;
            private final State state;
            private final Rational cost;
            private final Set<Flaw> preconditions = new HashSet<>();
            private boolean current = false;

            private Resolver(final long id, final Flaw effect, final String label, final State state,
                    final Rational cost) {
                this.id = id;
                this.effect = effect;
                this.label = label;
                this.state = state;
                this.cost = cost;
            }
        }

        private static class ResolverSerializer extends StdSerializer<Resolver> {

            private ResolverSerializer() {
                super(Resolver.class);
            }

            /**
             *
             */
            private static final long serialVersionUID = 1L;

            @Override
            public void serialize(final Resolver resolver, final JsonGenerator gen, final SerializerProvider provider)
                    throws IOException {
                gen.writeStartObject();
                gen.writeNumberField("id", resolver.id);
                gen.writeNumberField("effect", resolver.effect.id);
                gen.writeStringField("label", resolver.label);
                gen.writeNumberField("state", resolver.state.ordinal());

                gen.writeObjectField("cost", resolver.cost);

                gen.writeArrayFieldStart("preconditions");
                resolver.preconditions.stream().forEach(pre -> {
                    try {
                        gen.writeNumber(pre.id);
                    } catch (final IOException e) {
                        LOG.error("Cannot serialize", e);
                    }
                });
                gen.writeEndArray();

                gen.writeBooleanField("current", resolver.current);

                gen.writeEndObject();
            }
        }

        @SuppressWarnings({ "unused" })
        @JsonTypeInfo(use = JsonTypeInfo.Id.NAME, include = JsonTypeInfo.As.PROPERTY, property = "message_type")
        @JsonSubTypes({ @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.Log.class, name = "log"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.StartedSolving.class, name = "started_solving"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.SolutionFound.class, name = "solution_found"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.InconsistentProblem.class, name = "inconsistent_problem"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.Graph.class, name = "graph"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.FlawCreated.class, name = "flaw_created"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.FlawStateChanged.class, name = "flaw_state_changed"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.FlawCostChanged.class, name = "flaw_cost_changed"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.FlawPositionChanged.class, name = "flaw_position_changed"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.CurrentFlaw.class, name = "current_flaw"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.ResolverCreated.class, name = "resolver_created"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.ResolverStateChanged.class, name = "resolver_state_changed"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.CurrentResolver.class, name = "current_resolver"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.CausalLinkAdded.class, name = "causal_link_added"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.Timelines.class, name = "timelines"),
                @com.fasterxml.jackson.annotation.JsonSubTypes.Type(value = Message.Tick.class, name = "tick") })
        static abstract class Message {

            public final String plan_id;

            Message(final String plan_id) {
                this.plan_id = plan_id;
            }

            static class Log extends Message {

                public final String log;

                Log(final String plan_id, final String log) {
                    super(plan_id);
                    this.log = log;
                }
            }

            static class StartedSolving extends Message {

                StartedSolving(final String plan_id) {
                    super(plan_id);
                }
            }

            static class SolutionFound extends Message {

                SolutionFound(final String plan_id) {
                    super(plan_id);
                }
            }

            static class InconsistentProblem extends Message {

                InconsistentProblem(final String plan_id) {
                    super(plan_id);
                }
            }

            static class Graph extends Message {

                public final Collection<Flaw> flaws;
                public final Collection<Resolver> resolvers;

                Graph(final String plan_id, final Collection<Flaw> flaws, final Collection<Resolver> resolvers) {
                    super(plan_id);
                    this.flaws = flaws;
                    this.resolvers = resolvers;
                }
            }

            static class FlawCreated extends Message {

                public final long id;
                public final long[] causes;
                public final String label;
                public final byte state;
                public final Bound position;

                FlawCreated(final String plan_id, final long id, final long[] causes, final String label,
                        final byte state, final Bound position) {
                    super(plan_id);
                    this.id = id;
                    this.causes = causes;
                    this.label = label;
                    this.state = state;
                    this.position = position;
                }
            }

            static class FlawStateChanged extends Message {

                public final long id;
                public final byte state;

                FlawStateChanged(final String plan_id, final long id, final byte state) {
                    super(plan_id);
                    this.id = id;
                    this.state = state;
                }
            }

            static class FlawCostChanged extends Message {

                public final long id;
                public final Rational cost;

                FlawCostChanged(final String plan_id, final long id, final Rational cost) {
                    super(plan_id);
                    this.id = id;
                    this.cost = cost;
                }
            }

            static class FlawPositionChanged extends Message {

                public final long id;
                public final Bound position;

                FlawPositionChanged(final String plan_id, final long id, final Bound position) {
                    super(plan_id);
                    this.id = id;
                    this.position = position;
                }
            }

            static class CurrentFlaw extends Message {

                public final long id;

                CurrentFlaw(final String plan_id, final long id) {
                    super(plan_id);
                    this.id = id;
                }
            }

            static class ResolverCreated extends Message {

                public final long id;
                public final long effect;
                public final Rational cost;
                public final String label;
                public final byte state;

                ResolverCreated(final String plan_id, final long id, final long effect, final Rational cost,
                        final String label, final byte state) {
                    super(plan_id);
                    this.id = id;
                    this.effect = effect;
                    this.cost = cost;
                    this.label = label;
                    this.state = state;
                }
            }

            static class ResolverStateChanged extends Message {

                public final long id;
                public final byte state;

                ResolverStateChanged(final String plan_id, final long id, final byte state) {
                    super(plan_id);
                    this.id = id;
                    this.state = state;
                }
            }

            static class CurrentResolver extends Message {

                public final long id;

                CurrentResolver(final String plan_id, final long id) {
                    super(plan_id);
                    this.id = id;
                }
            }

            static class CausalLinkAdded extends Message {

                public final long flaw;
                public final long resolver;

                CausalLinkAdded(final String plan_id, final long flaw, final long resolver) {
                    super(plan_id);
                    this.flaw = flaw;
                    this.resolver = resolver;
                }
            }

            static class Timelines extends Message {

                public final Collection<Object> timelines;

                Timelines(final String plan_id, final Collection<Object> timelines) {
                    super(plan_id);
                    this.timelines = timelines;
                }
            }

            static class Tick extends Message {

                public final Rational current_time;

                Tick(final String plan_id, final Rational current_time) {
                    super(plan_id);
                    this.current_time = current_time;
                }
            }
        }
    }
}
