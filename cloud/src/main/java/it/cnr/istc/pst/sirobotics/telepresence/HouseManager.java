package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import javax.persistence.EntityManager;

import com.fasterxml.jackson.core.JsonProcessingException;

import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Item.EnumItem;
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
    private ScheduledFuture<?> scheduled_feature;
    private HouseEntity house;
    private MqttClient mqtt_client;

    public HouseManager(final HouseEntity house, final MqttClient mqtt_client) {
        this.house = house;
        this.mqtt_client = mqtt_client;
        final Map<Long, Atom> atoms = new HashMap<>();
        final Solver solver = new Solver();
        solver.addStateListener(new StateListener() {

            @Override
            public void inconsistentProblem() {
                try {
                    mqtt_client.publish(house.getId() + "/planner/out", "inconsistent".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void log(final String log) {
                LOG.info("[House #" + house.getId() + "] " + log);
            }

            @Override
            public void read(final String script) {
            }

            @Override
            public void read(final String[] files) {
            }

            @Override
            public void solutionFound() {
                try {
                    mqtt_client.publish(house.getId() + "/planner/out", "solution".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }

                atoms.clear();

                for (final Type t : solver.getTypes().values())
                    for (final Predicate p : t.getPredicates().values())
                        p.getInstances().stream().map(atm -> (Atom) atm)
                                .filter(atm -> (atm.getState() == Atom.AtomState.Active))
                                .forEach(atm -> atoms.put(atm.getSigma(), atm));
            }

            @Override
            public void startedSolving() {
                try {
                    mqtt_client.publish(house.getId() + "/planner/out", "solving".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void stateChanged() {
            }
        });
        final TimelinesExecutor tl_exec = new TimelinesExecutor(solver, new Rational(1));
        tl_exec.addExecutorListener(new ExecutorListener() {

            @Override
            public void endingAtoms(final long[] atms) {
                try {
                    mqtt_client.publish(house.getId() + "/planner/out/ending",
                            App.MAPPER.writeValueAsString(atms).getBytes(), App.QoS, false);
                } catch (final JsonProcessingException | MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void startingAtoms(final long[] atms) {
                final Command[] commands = new Command[atms.length];
                for (int i = 0; i < commands.length; i++)
                    commands[i] = new Command(atoms.get(atms[i]));
                try {
                    mqtt_client.publish(house.getId() + "/planner/out/starting",
                            App.MAPPER.writeValueAsString(commands).getBytes(), App.QoS, false);
                } catch (final JsonProcessingException | MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void tick(final Rational current_time) {
                try {
                    if (((ArithItem) solver.get("horizon")).getValue().leq(current_time)) {
                        LOG.info("Nothing more to execute..");
                        scheduled_feature.cancel(false);
                    }
                } catch (final NoSuchFieldException e) {
                    e.printStackTrace();
                }
            }
        });
        try {
            mqtt_client.subscribe(house.getId() + "/planner/in/plan", (topic, message) -> {
                // we read the problem..
                LOG.info("Reading the problem..");
                solver.read(new String(message.getPayload()));

                // we solve the problem..
                LOG.info("Solving the problem..");
                solver.solve();

                // we execute the solution..
                LOG.info("Starting plan execution..");
                scheduled_feature = EXECUTOR.scheduleAtFixedRate(() -> tl_exec.tick(), 0, 1000, TimeUnit.SECONDS);
            });

            mqtt_client.subscribe(house.getId() + "/planner/in/dont-start-yet", (topic, message) -> tl_exec
                    .dont_start_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            mqtt_client.subscribe(house.getId() + "/planner/in/dont-end-yet", (topic, message) -> tl_exec
                    .dont_end_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));
        } catch (final MqttException ex) {
            LOG.error("Cannot subscribe the house #" + house.getId() + " to the MQTT broker..", ex);
        }
        for (final DeviceEntity device_entity : house.getDevices()) {
            if (device_entity instanceof RobotEntity) {
                addRobot((RobotEntity) device_entity);
            } else if (device_entity instanceof SensorEntity) {
                addSensor((SensorEntity) device_entity);
            }
        }
    }

    public void addRobot(RobotEntity robot_entity) {
        final Map<Long, Atom> atoms = new HashMap<>();
        final Solver solver = new Solver();
        solver.addStateListener(new StateListener() {

            @Override
            public void inconsistentProblem() {
                try {
                    mqtt_client.publish(house.getId() + "/" + robot_entity.getId() + "/planner/out",
                            "inconsistent".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void log(final String log) {
                LOG.info("[House #" + house.getId() + ", Robot #" + robot_entity.getId() + "] " + log);
            }

            @Override
            public void read(final String script) {
            }

            @Override
            public void read(final String[] files) {
            }

            @Override
            public void solutionFound() {
                try {
                    mqtt_client.publish(house.getId() + "/" + robot_entity.getId() + "/planner/out",
                            "solution".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }

                atoms.clear();

                for (final Type t : solver.getTypes().values())
                    for (final Predicate p : t.getPredicates().values())
                        p.getInstances().stream().map(atm -> (Atom) atm)
                                .filter(atm -> (atm.getState() == Atom.AtomState.Active))
                                .forEach(atm -> atoms.put(atm.getSigma(), atm));
            }

            @Override
            public void startedSolving() {
                try {
                    mqtt_client.publish(house.getId() + "/" + robot_entity.getId() + "/planner/out",
                            "solving".getBytes(), App.QoS, false);
                } catch (final MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void stateChanged() {
            }
        });
        final TimelinesExecutor tl_exec = new TimelinesExecutor(solver, new Rational(1));
        tl_exec.addExecutorListener(new ExecutorListener() {

            @Override
            public void endingAtoms(final long[] atms) {
                try {
                    mqtt_client.publish(house.getId() + "/" + robot_entity.getId() + "/planner/out/ending",
                            App.MAPPER.writeValueAsString(atms).getBytes(), App.QoS, false);
                } catch (final JsonProcessingException | MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void startingAtoms(final long[] atms) {
                final Command[] commands = new Command[atms.length];
                for (int i = 0; i < commands.length; i++)
                    commands[i] = new Command(atoms.get(atms[i]));
                try {
                    mqtt_client.publish(house.getId() + "/" + robot_entity.getId() + "/planner/out/starting",
                            App.MAPPER.writeValueAsString(commands).getBytes(), App.QoS, false);
                } catch (final JsonProcessingException | MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void tick(final Rational current_time) {
                try {
                    if (((ArithItem) solver.get("horizon")).getValue().leq(current_time)) {
                        LOG.info("Nothing more to execute..");
                        scheduled_feature.cancel(false);
                    }
                } catch (final NoSuchFieldException e) {
                    e.printStackTrace();
                }
            }
        });
        try {
            mqtt_client.subscribe(house.getId() + "/" + robot_entity.getId() + "/planner/in/plan", (topic, message) -> {
                // we read the problem..
                LOG.info("Reading the problem..");
                solver.read(new String(message.getPayload()));

                // we solve the problem..
                LOG.info("Solving the problem..");
                solver.solve();

                // we execute the solution..
                LOG.info("Starting plan execution..");
                scheduled_feature = EXECUTOR.scheduleAtFixedRate(() -> tl_exec.tick(), 0, 1000, TimeUnit.SECONDS);
            });

            mqtt_client.subscribe(house.getId() + "/" + robot_entity.getId() + "/planner/in/dont-start-yet",
                    (topic, message) -> tl_exec
                            .dont_start_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            mqtt_client.subscribe(house.getId() + "/" + robot_entity.getId() + "/planner/in/dont-end-yet",
                    (topic, message) -> tl_exec
                            .dont_end_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));
        } catch (final MqttException ex) {
            LOG.error("Cannot subscribe the robot #" + robot_entity.getId() + " to the MQTT broker..", ex);
        }
    }

    public void addSensor(SensorEntity sensor_entity) {
        try {
            mqtt_client.subscribe(house.getId() + "/" + sensor_entity.getId(), (topic, message) -> {
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

    @SuppressWarnings("unused")
    private static final class Command {

        private final long id;
        private final String predicate;
        private final Collection<String> bool_pars = new ArrayList<>();
        private final Collection<String> bool_vals = new ArrayList<>();
        private final Collection<String> arith_pars = new ArrayList<>();
        private final Collection<Double> arith_vals = new ArrayList<>();
        private final Collection<String> string_pars = new ArrayList<>();
        private final Collection<String> string_vals = new ArrayList<>();
        private final Collection<String> enum_pars = new ArrayList<>();
        private final Collection<Collection<String>> enum_vals = new ArrayList<>();

        private Command(final Atom atom) {
            this.id = atom.getSigma();
            this.predicate = atom.getType().getName();

            atom.getExprs().entrySet().forEach(expr -> {
                switch (expr.getValue().getType().getName()) {
                case Solver.BOOL:
                    bool_pars.add(expr.getKey());
                    bool_vals.add(((Item.BoolItem) expr.getValue()).getValue().name());
                case Solver.INT:
                case Solver.REAL:
                case Solver.TP:
                    arith_pars.add(expr.getKey());
                    arith_vals.add(((Item.ArithItem) expr.getValue()).getValue().doubleValue());
                case Solver.STRING:
                    string_pars.add(expr.getKey());
                    string_vals.add(((Item.StringItem) expr.getValue()).getValue());
                default:
                    enum_pars.add(expr.getKey());
                    if (expr.getValue() instanceof EnumItem) {
                        if (((EnumItem) expr.getValue()).getVals().length == 1)
                            enum_vals.add(Collections.singleton(((EnumItem) expr.getValue()).getVals()[0].getName()));
                        else
                            enum_vals.add(Stream.of(((EnumItem) expr.getValue()).getVals()).map(itm -> itm.getName())
                                    .collect(Collectors.toList()));
                    } else
                        enum_vals.add(Collections.singleton(expr.getValue().getName()));
                }
            });
        }
    }
}
