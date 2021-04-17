package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javax.persistence.EntityManager;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;

import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Item.BoolItem;
import it.cnr.istc.pst.oratio.Item.EnumItem;
import it.cnr.istc.pst.oratio.Item.StringItem;
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
    private final HouseEntity house;

    public HouseManager(final HouseEntity house) {
        this.house = house;
        final Map<Long, Atom> atoms = new HashMap<>();
        final Solver solver = new Solver();
        solver.addStateListener(new StateListener() {

            @Override
            public void inconsistentProblem() {
                try {
                    App.MQTT_CLIENT.publish(house.getId() + "/planner", "inconsistent".getBytes(), App.QoS, false);
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
                    App.MQTT_CLIENT.publish(house.getId() + "/planner", "solution".getBytes(), App.QoS, false);
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
                    App.MQTT_CLIENT.publish(house.getId() + "/planner", "solving".getBytes(), App.QoS, false);
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
                    App.MQTT_CLIENT.publish(house.getId() + "/ending", App.MAPPER.writeValueAsString(atms).getBytes(),
                            App.QoS, false);
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
                    App.MQTT_CLIENT.publish(house.getId() + "/starting",
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
            App.MQTT_CLIENT.subscribe(house.getId() + "/plan", (topic, message) -> {
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

            App.MQTT_CLIENT.subscribe(house.getId() + "/dont_start_yet", (topic, message) -> tl_exec
                    .dont_start_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/dont_end_yet", (topic, message) -> tl_exec
                    .dont_end_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/failure", (topic, message) -> tl_exec
                    .failure(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/nlp/in", (topic, message) -> {
                JsonNode parse = App.NLU_CLIENT.parse(new String(message.getPayload()));
                App.MQTT_CLIENT.publish(house.getId() + "/nlp/intent", parse.get("intent").toString().getBytes(),
                        App.QoS, true);
            });
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

    public void addRobot(final RobotEntity robot_entity) {
        final Map<Long, Atom> atoms = new HashMap<>();
        final Solver solver = new Solver();
        solver.addStateListener(new StateListener() {

            @Override
            public void inconsistentProblem() {
                try {
                    App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + "/planner",
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
                    App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + "/planner",
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
                    App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + "/planner",
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
                    App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + "/ending",
                            App.MAPPER.writeValueAsString(atms).getBytes(), App.QoS, false);
                } catch (final JsonProcessingException | MqttException ex) {
                    LOG.error("Cannot create MQTT message..", ex);
                }
            }

            @Override
            public void startingAtoms(final long[] atms) {
                final Map<String, Collection<Command>> commands = new HashMap<>();
                for (int i = 0; i < atms.length; i++) {
                    Atom atm = atoms.get(atms[i]);
                    Collection<Command> c_cmnds = commands.putIfAbsent(atm.getType().getName(), new ArrayList<>());
                    c_cmnds.add(new Command(atm));
                }
                for (Map.Entry<String, Collection<Command>> cmnd : commands.entrySet()) {
                    try {
                        App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + '/' + cmnd.getKey(),
                                App.MAPPER.writeValueAsString(cmnd.getValue()).getBytes(), App.QoS, false);
                    } catch (final JsonProcessingException | MqttException ex) {
                        LOG.error("Cannot create MQTT message..", ex);
                    }
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
            App.MQTT_CLIENT.subscribe(house.getId() + "/" + robot_entity.getId() + "/plan", (topic, message) -> {
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

            App.MQTT_CLIENT.subscribe(house.getId() + "/" + robot_entity.getId() + "/dont_start_yet",
                    (topic, message) -> tl_exec
                            .dont_start_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/" + robot_entity.getId() + "/dont_end_yet",
                    (topic, message) -> tl_exec
                            .dont_end_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/" + robot_entity.getId() + "/failure", (topic,
                    message) -> tl_exec.failure(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            App.MQTT_CLIENT.subscribe(house.getId() + "/" + robot_entity.getId() + "/nlp/in", (topic, message) -> {
                JsonNode parse = App.NLU_CLIENT.parse(new String(message.getPayload()));
                App.MQTT_CLIENT.publish(house.getId() + "/" + robot_entity.getId() + "/nlp/intent",
                        parse.get("intent").toString().getBytes(), App.QoS, true);
            });
        } catch (final MqttException ex) {
            LOG.error("Cannot subscribe the robot #" + robot_entity.getId() + " to the MQTT broker..", ex);
        }
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

    @SuppressWarnings("unused")
    private static final class Command {

        private final long id;
        private final List<String> pars = new ArrayList<>();
        private final List<Object> vals = new ArrayList<>();

        private Command(final Atom atom) {
            this.id = atom.getSigma();

            atom.getExprs().entrySet().forEach(expr -> {
                switch (expr.getValue().getType().getName()) {
                case Solver.BOOL:
                    pars.add(expr.getKey());
                    if (expr.getValue() instanceof EnumItem)
                        vals.add(((BoolItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue().booleanValue());
                    else
                        vals.add(((BoolItem) expr.getValue()).getValue().booleanValue());
                    break;
                case Solver.INT:
                case Solver.REAL:
                case Solver.TP:
                    pars.add(expr.getKey());
                    if (expr.getValue() instanceof EnumItem)
                        vals.add(((ArithItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue().doubleValue());
                    else
                        vals.add(((ArithItem) expr.getValue()).getValue().doubleValue());
                    break;
                case Solver.STRING:
                    pars.add(expr.getKey());
                    if (expr.getValue() instanceof EnumItem)
                        vals.add(((StringItem) ((EnumItem) expr.getValue()).getVals()[0]).getValue());
                    else
                        vals.add(((StringItem) expr.getValue()).getValue());
                    break;
                default:
                    pars.add(expr.getKey());
                    if (expr.getValue() instanceof EnumItem)
                        vals.add(((EnumItem) expr.getValue()).getVals()[0].getName());
                    else
                        vals.add(expr.getValue().getName());
                    break;
                }
            });
        }
    }
}
