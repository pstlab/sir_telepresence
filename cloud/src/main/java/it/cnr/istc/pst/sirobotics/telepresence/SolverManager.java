package it.cnr.istc.pst.sirobotics.telepresence;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.persistence.EntityManager;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.SerializerProvider;
import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import com.fasterxml.jackson.databind.ser.std.StdSerializer;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.Bound;
import it.cnr.istc.pst.oratio.GraphListener;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Item.BoolItem;
import it.cnr.istc.pst.oratio.Item.EnumItem;
import it.cnr.istc.pst.oratio.Item.StringItem;
import it.cnr.istc.pst.oratio.Predicate;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.SolverException;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.Type;
import it.cnr.istc.pst.oratio.timelines.ExecutorException;
import it.cnr.istc.pst.oratio.timelines.ExecutorListener;
import it.cnr.istc.pst.oratio.timelines.Timeline;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.oratio.utils.Flaw;
import it.cnr.istc.pst.oratio.utils.Resolver;
import it.cnr.istc.pst.sirobotics.telepresence.api.ExecConf;
import it.cnr.istc.pst.sirobotics.telepresence.api.LongArray;
import it.cnr.istc.pst.sirobotics.telepresence.api.StringWrapper;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class SolverManager implements StateListener, GraphListener, ExecutorListener {

    private static final Logger LOG = LoggerFactory.getLogger(SolverManager.class);
    private static final ScheduledExecutorService EXECUTOR = Executors
            .newScheduledThreadPool(Runtime.getRuntime().availableProcessors());
    private static final Executor COMMAND_EXECUTOR = Executors.newSingleThreadExecutor();
    private final String prefix;
    private Solver solver = null;
    private final Map<Long, Atom> c_atoms = new HashMap<>();
    private TimelinesExecutor tl_exec = null;
    private ScheduledFuture<?> scheduled_feature;
    private final Map<Long, Flaw> flaws = new HashMap<>();
    private Flaw current_flaw = null;
    private final Map<Long, Resolver> resolvers = new HashMap<>();
    private Resolver current_resolver = null;
    private Rational current_time = new Rational();
    private ExecConf conf;
    private final Set<Predicate> notify_starting = new HashSet<>();
    private final Set<Predicate> notify_ending = new HashSet<>();
    private final Set<Predicate> auto_done = new HashSet<>();
    private final Set<Atom> executing = new HashSet<>();
    private SolverState state = null;

    public SolverManager(final String prefix, final String configuration) {
        this.prefix = prefix;

        try {
            conf = App.MAPPER.readValue(configuration, ExecConf.class);
        } catch (final JsonProcessingException ex) {
            LOG.error("Cannot read config file..", ex);
        }

        try {
            App.MQTT_CLIENT.subscribe(prefix + "/plan", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    if (state != SolverState.Waiting) {
                        LOG.info("[" + prefix + "] Solver state is " + state.name()
                                + " and cannot read a new problem..");
                        return;
                    }

                    setState(SolverState.Solving);

                    // we read the problem..
                    LOG.info("[" + prefix + "] Reading the problem..");
                    try {
                        final StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                                StringWrapper.class);
                        solver.read(string_message.data);
                    } catch (IOException | SolverException e) {
                        LOG.error("Cannot solve the given problem", e);
                    }

                    // we solve the problem..
                    LOG.info("[" + prefix + "] Solving the problem..");
                    try {
                        solver.solve();
                    } catch (final SolverException e) {
                        LOG.error("Cannot solve the given problem", e);
                    }
                });
            });

            App.MQTT_CLIENT.subscribe(prefix + "/done", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    LOG.info("[" + prefix + "] Closing " + message.toString() + "..");
                    try {
                        final LongArray long_array_message = App.MAPPER.readValue(message.getPayload(),
                                LongArray.class);
                        for (final long atm_sgm : long_array_message.data)
                            executing.remove(c_atoms.get(atm_sgm));
                    } catch (final IOException e) {
                        LOG.error("Cannot parse the closing command", e);
                    }
                });
            });

            App.MQTT_CLIENT.subscribe(prefix + "/failure", (topic, message) -> {
                COMMAND_EXECUTOR.execute(() -> {
                    LOG.info("[" + prefix + "] Failing " + message.toString() + "..");
                    try {
                        final LongArray long_array_message = App.MAPPER.readValue(message.getPayload(),
                                LongArray.class);
                        LOG.info("[" + prefix + "] Failed " + long_array_message.data + "..");
                        tl_exec.failure(long_array_message.data);
                    } catch (IOException | ExecutorException e) {
                        LOG.error("Cannot fail the given commands", e);
                    }
                });
            });
        } catch (final MqttException ex) {
            LOG.error("Cannot subscribe the solver #" + prefix + " to the MQTT broker..", ex);
        }

        reset();
    }

    public Solver getSolver() {
        return solver;
    }

    public Collection<Flaw> getFlaws() {
        return flaws.values();
    }

    public Flaw getCurrentFlaw() {
        return current_flaw;
    }

    public Collection<Resolver> getResolvers() {
        return resolvers.values();
    }

    public Resolver getCurrentResolver() {
        return current_resolver;
    }

    public Rational getCurrentTime() {
        return current_time;
    }

    public SolverState getState() {
        return state;
    }

    public void setState(final SolverState state) {
        this.state = state;
        fireStateChanged();
    }

    public void reset() {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Resetting the solver..");
            flaws.clear();
            resolvers.clear();
            current_time = new Rational();
            solver = new Solver();
            solver.addStateListener(this);
            solver.addGraphListener(this);
            tl_exec = new TimelinesExecutor(solver,
                    "{ \"relevant-predicates\": ["
                            + conf.getRelevantPredicates().stream().collect(Collectors.joining(", ")) + "]}",
                    new Rational(1));
            tl_exec.addExecutorListener(this);
            notify_starting.clear();
            notify_ending.clear();
            auto_done.clear();
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId())) {
                        final WsContext ctx = UserController.getWsContext(ue.getId());
                        ctx.send(App.MAPPER.writeValueAsString(new Graph(prefix, flaws.values(), resolvers.values())));
                        ctx.send(App.MAPPER.writeValueAsString(new Timelines(prefix, solver.getTimelines())));
                        ctx.send(App.MAPPER.writeValueAsString(new Tick(prefix, current_time)));
                    }
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
            setState(SolverState.Waiting);
        });
    }

    @Override
    public void log(final String log) {
        LOG.info("[" + prefix + "] " + log);
    }

    private Predicate getPredicate(final String predicate) throws ClassNotFoundException {
        final String[] pred = predicate.split(".");
        if (pred.length == 1)
            return solver.getPredicate(pred[0]);
        else {
            Type t = solver.getType(pred[0]);
            for (int i = 1; i < pred.length - 1; i++)
                t = t.getType(pred[i]);
            return t.getPredicate(pred[pred.length - 1]);
        }
    }

    @Override
    public void read(final String script) {
        COMMAND_EXECUTOR.execute(() -> {
            for (final String predicate : conf.getNotifyStarting())
                try {
                    notify_starting.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
            for (final String predicate : conf.getNotifyEnding())
                try {
                    notify_ending.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
            for (final String predicate : conf.getAutoDone())
                try {
                    auto_done.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
        });
    }

    @Override
    public void read(final String[] files) {
        COMMAND_EXECUTOR.execute(() -> {
            for (final String predicate : conf.getNotifyStarting())
                try {
                    notify_starting.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
            for (final String predicate : conf.getNotifyEnding())
                try {
                    notify_ending.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
            for (final String predicate : conf.getAutoDone())
                try {
                    auto_done.add(getPredicate(predicate));
                } catch (final ClassNotFoundException e) {
                    LOG.info("Cannot find predicate {}", predicate);
                }
        });
    }

    @Override
    public void stateChanged() {
    }

    @Override
    public void startedSolving() {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Solving the problem..");
            setState(SolverState.Solving);
        });
    }

    @Override
    public void solutionFound() {
        COMMAND_EXECUTOR.execute(() -> {
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
                }, 1, 1, TimeUnit.SECONDS);
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
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId())) {
                        final WsContext ctx = UserController.getWsContext(ue.getId());
                        ctx.send(App.MAPPER.writeValueAsString(new Timelines(prefix, solver.getTimelines())));
                        ctx.send(App.MAPPER.writeValueAsString(new SolutionFound(prefix)));
                        ctx.send(App.MAPPER.writeValueAsString(new Tick(prefix, current_time)));
                    }
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
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
        COMMAND_EXECUTOR.execute(() -> {
            final Flaw c_flaw = new Flaw(id,
                    Arrays.stream(causes).mapToObj(r_id -> resolvers.get(r_id)).toArray(Resolver[]::new), label, state,
                    position);
            Stream.of(c_flaw.causes).forEach(c -> c.preconditions.add(c_flaw));
            flaws.put(id, c_flaw);
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId()).send(App.MAPPER.writeValueAsString(
                                new FlawCreated(prefix, id, causes, label, (byte) state.ordinal(), position)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawStateChanged(final long id, final State state) {
        COMMAND_EXECUTOR.execute(() -> {
            final Flaw flaw = flaws.get(id);
            flaw.state = state;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId()).send(App.MAPPER
                                .writeValueAsString(new FlawStateChanged(prefix, id, (byte) state.ordinal())));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawCostChanged(final long id, final Rational cost) {
        COMMAND_EXECUTOR.execute(() -> {
            final Flaw flaw = flaws.get(id);
            flaw.cost = cost;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new FlawCostChanged(prefix, id, cost)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void flawPositionChanged(final long id, final Bound position) {
        COMMAND_EXECUTOR.execute(() -> {
            final Flaw flaw = flaws.get(id);
            flaw.position = position;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new FlawPositionChanged(prefix, id, position)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void currentFlaw(final long id) {
        COMMAND_EXECUTOR.execute(() -> {
            if (current_flaw != null)
                current_flaw.current = false;
            final Flaw flaw = flaws.get(id);
            current_flaw = flaw;
            current_flaw.current = true;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new CurrentFlaw(prefix, id)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void resolverCreated(final long id, final long effect, final Rational cost, final String label,
            final State state) {
        COMMAND_EXECUTOR.execute(() -> {
            final Resolver resolver = new Resolver(id, flaws.get(effect), label, state, cost);
            resolvers.put(id, resolver);
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId()).send(App.MAPPER.writeValueAsString(
                                new ResolverCreated(prefix, id, effect, cost, label, (byte) state.ordinal())));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void resolverStateChanged(final long id, final State state) {
        COMMAND_EXECUTOR.execute(() -> {
            final Resolver resolver = resolvers.get(id);
            resolver.state = state;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId()).send(App.MAPPER
                                .writeValueAsString(new ResolverStateChanged(prefix, id, (byte) state.ordinal())));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void currentResolver(final long id) {
        COMMAND_EXECUTOR.execute(() -> {
            if (current_resolver != null)
                current_resolver.current = false;
            final Resolver resolver = resolvers.get(id);
            current_resolver = resolver;
            current_resolver.current = true;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new CurrentResolver(prefix, id)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void causalLinkAdded(final long flaw, final long resolver) {
        COMMAND_EXECUTOR.execute(() -> {
            resolvers.get(resolver).preconditions.add(flaws.get(flaw));
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new CausalLinkAdded(prefix, flaw, resolver)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
        });
    }

    @Override
    public void tick(final Rational current_time) {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Current time: " + current_time.toString());
            this.current_time = current_time;
            try {
                final EntityManager em = App.EMF.createEntityManager();
                final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();
                for (final UserEntity ue : user_entities)
                    if (ue.getRoles().contains("Admin") && UserController.isOnline(ue.getId()))
                        UserController.getWsContext(ue.getId())
                                .send(App.MAPPER.writeValueAsString(new Tick(prefix, current_time)));
                em.close();
            } catch (final JsonProcessingException e) {
                LOG.error("Cannot serialize", e);
            }
            try {
                if (((ArithItem) solver.get("horizon")).getValue().leq(current_time)) {
                    LOG.info("Nothing more to execute..");
                    scheduled_feature.cancel(false);
                    reset();
                }
            } catch (final NoSuchFieldException e) {
                e.printStackTrace();
            }
        });
    }

    @Override
    public void startingAtoms(final long[] atoms) {
    }

    @Override
    public void startAtoms(final long[] atoms) {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Starting atoms: " + Arrays.toString(atoms));
            final Map<Type, Collection<Atom>> starting_atoms = new IdentityHashMap<>();
            for (int i = 0; i < atoms.length; i++) {
                final Atom starting_atom = c_atoms.get(atoms[i]);
                if (notify_starting.contains(starting_atom.getType())) {
                    Collection<Atom> c_atms = starting_atoms.get(starting_atom.getType());
                    if (c_atms == null) {
                        c_atms = new ArrayList<>();
                        starting_atoms.put(starting_atom.getType(), c_atms);
                    }
                    c_atms.add(starting_atom);
                }
                executing.add(starting_atom);
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
        });
    }

    @Override
    public void endingAtoms(final long[] atoms) {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Checking the ending of atoms: {}" + Arrays.toString(atoms));
            final List<Atom> dont_end_yet = new ArrayList<>();
            for (int i = 0; i < atoms.length; i++) {
                final Atom ending_atom = c_atoms.get(atoms[i]);
                if (auto_done.contains(ending_atom.getType()))
                    executing.remove(ending_atom);
                else if (executing.contains(ending_atom))
                    dont_end_yet.add(ending_atom);
            }
            if (!dont_end_yet.isEmpty()) {
                final long[] dont_e = new long[dont_end_yet.size()];
                for (int i = 0; i < dont_e.length; i++)
                    dont_e[i] = dont_end_yet.get(i).getSigma();
                LOG.info("[" + prefix + "] Delaying atoms: {}" + Arrays.toString(dont_e));
                try {
                    tl_exec.dont_end_yet(dont_e);
                } catch (final ExecutorException e) {
                    LOG.error("Cannot delay the ending of some commands..", e);
                }
            }
        });
    }

    @Override
    public void endAtoms(final long[] atoms) {
        COMMAND_EXECUTOR.execute(() -> {
            LOG.info("[" + prefix + "] Ending atoms: {}" + Arrays.toString(atoms));
            final Map<Type, Collection<Atom>> ending_atoms = new IdentityHashMap<>();
            final List<Atom> done_atoms = new ArrayList<>();
            for (int i = 0; i < atoms.length; i++) {
                final Atom ending_atom = c_atoms.get(atoms[i]);
                if (notify_ending.contains(ending_atom.getType())) {
                    Collection<Atom> c_atms = ending_atoms.get(ending_atom.getType());
                    if (c_atms == null) {
                        c_atms = new ArrayList<>();
                        ending_atoms.put(ending_atom.getType(), c_atms);
                    }
                    c_atms.add(ending_atom);
                    done_atoms.add(ending_atom);
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
        });
    }

    private void fireStateChanged() {
        COMMAND_EXECUTOR.execute(() -> {
            try {
                App.MQTT_CLIENT.publish(prefix + "/planner",
                        App.MAPPER.writeValueAsString(new StringWrapper(state.name())).getBytes(), App.QoS, true);
            } catch (final MqttException | JsonProcessingException ex) {
                LOG.error("Cannot create MQTT message..", ex);
            }
        });
    }

    public enum SolverState {
        Waiting, Solving, Solution, Inconsistent, Executing;
    }

    private static final class Command {

        @SuppressWarnings("unused")
        private final Collection<CommandAtom> atoms;

        private Command(final Collection<CommandAtom> atoms) {
            this.atoms = atoms;
        }
    }

    @JsonSerialize(using = CommandAtomSerializer.class)
    private static final class CommandAtom {

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

    static class CommandAtomSerializer extends StdSerializer<CommandAtom> {

        protected CommandAtomSerializer() {
            super(CommandAtom.class);
        }

        @Override
        public void serialize(final CommandAtom value, final JsonGenerator gen, final SerializerProvider provider)
                throws IOException {
            gen.writeStartObject();
            gen.writeNumberField("sigma", value.getAtom().getSigma());
            gen.writeBooleanField("starting", value.isStarting());
            gen.writeBooleanField("ending", value.isEnding());
            for (final Entry<String, Item> expr : value.getAtom().getExprs().entrySet()) {
                switch (expr.getValue().getType().getName()) {
                    case Solver.BOOL:
                        if (expr.getValue() instanceof EnumItem)
                            gen.writeBooleanField(expr.getKey(),
                                    ((BoolItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue().booleanValue());

                        else
                            gen.writeBooleanField(expr.getKey(),
                                    ((BoolItem) expr.getValue()).getValue().booleanValue());
                        break;
                    case Solver.INT:
                    case Solver.REAL:
                    case Solver.TP:
                        if (expr.getValue() instanceof EnumItem)
                            gen.writeNumberField(expr.getKey(),
                                    ((ArithItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue().doubleValue());
                        else
                            gen.writeNumberField(expr.getKey(), ((ArithItem) expr.getValue()).getValue().doubleValue());
                        break;
                    case Solver.STRING:
                        if (expr.getValue() instanceof EnumItem)
                            gen.writeStringField(expr.getKey(),
                                    ((StringItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue());
                        else
                            gen.writeStringField(expr.getKey(), ((StringItem) expr.getValue()).getValue());
                        break;
                    default:
                        if (expr.getValue() instanceof EnumItem)
                            gen.writeStringField(expr.getKey(), ((EnumItem) expr.getValue()).getVals()[0].getName());
                        else
                            gen.writeStringField(expr.getKey(), expr.getValue().getName());
                        break;
                }
            }
            gen.writeEndObject();
        }
    }

    static class Log extends it.cnr.istc.pst.oratio.utils.Message.Log {

        public final String plan_id;

        Log(final String plan_id, final String log) {
            super(log);
            this.plan_id = plan_id;
        }
    }

    static class StartedSolving extends it.cnr.istc.pst.oratio.utils.Message.StartedSolving {

        public final String plan_id;

        StartedSolving(final String plan_id) {
            this.plan_id = plan_id;
        }
    }

    static class SolutionFound extends it.cnr.istc.pst.oratio.utils.Message.SolutionFound {

        public final String plan_id;

        SolutionFound(final String plan_id) {
            this.plan_id = plan_id;
        }
    }

    static class InconsistentProblem extends it.cnr.istc.pst.oratio.utils.Message.InconsistentProblem {

        public final String plan_id;

        InconsistentProblem(final String plan_id) {
            this.plan_id = plan_id;
        }
    }

    static class Graph extends it.cnr.istc.pst.oratio.utils.Message.Graph {

        public final String plan_id;

        Graph(final String plan_id, final Collection<Flaw> flaws, final Collection<Resolver> resolvers) {
            super(flaws, resolvers);
            this.plan_id = plan_id;
        }
    }

    static class FlawCreated extends it.cnr.istc.pst.oratio.utils.Message.FlawCreated {

        public final String plan_id;

        FlawCreated(final String plan_id, final long id, final long[] causes, final String label, final byte state,
                final Bound position) {
            super(id, causes, label, state, position);
            this.plan_id = plan_id;
        }
    }

    static class FlawStateChanged extends it.cnr.istc.pst.oratio.utils.Message.FlawStateChanged {

        public final String plan_id;

        FlawStateChanged(final String plan_id, final long id, final byte state) {
            super(id, state);
            this.plan_id = plan_id;
        }
    }

    static class FlawCostChanged extends it.cnr.istc.pst.oratio.utils.Message.FlawCostChanged {

        public final String plan_id;

        FlawCostChanged(final String plan_id, final long id, final Rational cost) {
            super(id, cost);
            this.plan_id = plan_id;
        }
    }

    static class FlawPositionChanged extends it.cnr.istc.pst.oratio.utils.Message.FlawPositionChanged {

        public final String plan_id;

        FlawPositionChanged(final String plan_id, final long id, final Bound position) {
            super(id, position);
            this.plan_id = plan_id;
        }
    }

    static class CurrentFlaw extends it.cnr.istc.pst.oratio.utils.Message.CurrentFlaw {

        public final String plan_id;

        CurrentFlaw(final String plan_id, final long id) {
            super(id);
            this.plan_id = plan_id;
        }
    }

    static class ResolverCreated extends it.cnr.istc.pst.oratio.utils.Message.ResolverCreated {

        public final String plan_id;

        ResolverCreated(final String plan_id, final long id, final long effect, final Rational cost, final String label,
                final byte state) {
            super(id, effect, cost, label, state);
            this.plan_id = plan_id;
        }
    }

    static class ResolverStateChanged extends it.cnr.istc.pst.oratio.utils.Message.ResolverStateChanged {

        public final String plan_id;

        ResolverStateChanged(final String plan_id, final long id, final byte state) {
            super(id, state);
            this.plan_id = plan_id;
        }
    }

    static class CurrentResolver extends it.cnr.istc.pst.oratio.utils.Message.CurrentResolver {

        public final String plan_id;

        CurrentResolver(final String plan_id, final long id) {
            super(id);
            this.plan_id = plan_id;
        }
    }

    static class CausalLinkAdded extends it.cnr.istc.pst.oratio.utils.Message.CausalLinkAdded {

        public final String plan_id;

        CausalLinkAdded(final String plan_id, final long flaw, final long resolver) {
            super(flaw, resolver);
            this.plan_id = plan_id;
        }
    }

    static class Timelines extends it.cnr.istc.pst.oratio.utils.Message.Timelines {

        public final String plan_id;

        Timelines(final String plan_id, final Collection<Timeline<?>> timelines) {
            super(timelines);
            this.plan_id = plan_id;
        }
    }

    static class Tick extends it.cnr.istc.pst.oratio.utils.Message.Tick {

        public final String plan_id;

        Tick(final String plan_id, final Rational current_time) {
            super(current_time);
            this.plan_id = plan_id;
        }
    }
}
