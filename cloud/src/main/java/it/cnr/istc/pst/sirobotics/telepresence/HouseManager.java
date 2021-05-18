package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javax.persistence.EntityManager;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Predicate;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.StateListener;
import it.cnr.istc.pst.oratio.Type;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.sirobotics.telepresence.db.DeviceEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorDataEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorEntity;

public class HouseManager {

    private static final Logger LOG = LoggerFactory.getLogger(HouseManager.class);
    private static final ScheduledExecutorService EXECUTOR = Executors
            .newScheduledThreadPool(Runtime.getRuntime().availableProcessors());
    private final HouseEntity house;

    public HouseManager(final HouseEntity house) {
        this.house = house;
        new SolverManager(Long.toString(house.getId()));

        for (final DeviceEntity device_entity : house.getDevices()) {
            if (device_entity instanceof RobotEntity) {
                addRobot((RobotEntity) device_entity);
            } else if (device_entity instanceof SensorEntity) {
                addSensor((SensorEntity) device_entity);
            }
        }
    }

    public void addRobot(final RobotEntity robot_entity) {
        new SolverManager(house.getId() + "/" + robot_entity.getId());
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

    static class SolverManager implements StateListener, ExecutorListener {

        private final String prefix;
        private Solver solver = null;
        private final Map<Long, Atom> c_atoms = new HashMap<>();
        private TimelinesExecutor tl_exec = null;
        private ScheduledFuture<?> scheduled_feature;
        private SolverState state = null;

        public SolverManager(final String prefix) {
            this.prefix = prefix;

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
                    StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                            StringWrapper.class);
                    solver.read(string_message.data);

                    // we solve the problem..
                    LOG.info("[" + prefix + "] Solving the problem..");
                    solver.solve();
                });

                App.MQTT_CLIENT.subscribe(prefix + "/done", (topic, message) -> {
                    scheduled_feature.cancel(false);
                    LongArray long_array_message = App.MAPPER.readValue(message.getPayload(), LongArray.class);
                    tl_exec.done(long_array_message.data);
                });

                App.MQTT_CLIENT.subscribe(prefix + "/failure", (topic, message) -> {
                    scheduled_feature.cancel(false);
                    LongArray long_array_message = App.MAPPER.readValue(message.getPayload(), LongArray.class);
                    tl_exec.failure(long_array_message.data);
                });

                App.MQTT_CLIENT.subscribe(prefix + "/nlp/in", (topic, message) -> {
                    StringWrapper string_message = App.MAPPER.readValue(new String(message.getPayload()),
                            StringWrapper.class);
                    JsonNode parse = App.NLU_CLIENT.parse(string_message.data);
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

        public void setState(SolverState state) {
            this.state = state;
            fireStateChanged();
        }

        public void reset() {
            LOG.info("[" + prefix + "] Resetting the solver..");
            solver = new Solver();
            solver.addStateListener(this);
            tl_exec = new TimelinesExecutor(solver, new Rational(1));
            tl_exec.addExecutorListener(this);
            setState(SolverState.Waiting);
        }

        @Override
        public void log(String log) {
            LOG.info("[" + prefix + "] " + log);
        }

        @Override
        public void read(String script) {
        }

        @Override
        public void read(String[] files) {
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
            scheduled_feature = EXECUTOR.scheduleAtFixedRate(() -> tl_exec.tick(), 0, 1, TimeUnit.SECONDS);
        }

        @Override
        public void inconsistentProblem() {
            LOG.info("[" + prefix + "] Inconsistent problem..");
            setState(SolverState.Inconsistent);
            reset();
        }

        @Override
        public void tick(Rational current_time) {
            LOG.info("[" + prefix + "] Current time: " + current_time.toString());
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
        public void startingAtoms(long[] atoms) {
            LOG.info("[" + prefix + "] Starting atoms: " + Arrays.toString(atoms));
            final Map<Type, Collection<Atom>> starting_atoms = new IdentityHashMap<>();
            for (int i = 0; i < atoms.length; i++) {
                Atom starting_atom = c_atoms.get(atoms[i]);
                Collection<Atom> c_atms = starting_atoms.get(starting_atom.getType());
                if (c_atms == null) {
                    c_atms = new ArrayList<>();
                    starting_atoms.put(starting_atom.getType(), c_atms);
                }
                c_atms.add(starting_atom);
            }
            try {
                for (Entry<Type, Collection<Atom>> entry : starting_atoms.entrySet())
                    App.MQTT_CLIENT.publish(prefix + "/" + entry.getKey().getName(),
                            App.MAPPER.writeValueAsString(new Command(entry.getValue())).getBytes(), App.QoS, false);
            } catch (final JsonProcessingException | MqttException ex) {
                LOG.error("Cannot create MQTT message..", ex);
            }
        }

        @Override
        public void endingAtoms(long[] atoms) {
            LOG.info("[" + prefix + "] Ending atoms: {}" + Arrays.toString(atoms));
            try {
                App.MQTT_CLIENT.publish(prefix + "/ending",
                        App.MAPPER.writeValueAsString(new LongArray(atoms)).getBytes(), App.QoS, false);
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

            private final Collection<Atom> atoms;

            private Command(final Collection<Atom> atoms) {
                this.atoms = atoms;
            }
        }
    }
}
