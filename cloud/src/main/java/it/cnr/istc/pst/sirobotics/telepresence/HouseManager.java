package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.fasterxml.jackson.core.JsonProcessingException;

import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.cnr.istc.pst.oratio.Atom;
import it.cnr.istc.pst.oratio.ExecutorListener;
import it.cnr.istc.pst.oratio.Item;
import it.cnr.istc.pst.oratio.Item.ArithItem;
import it.cnr.istc.pst.oratio.Item.EnumItem;
import it.cnr.istc.pst.oratio.Rational;
import it.cnr.istc.pst.oratio.Solver;
import it.cnr.istc.pst.oratio.timelines.TimelinesExecutor;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;

public class HouseManager implements ExecutorListener {

    private static final String BROKER_URL = "tcp://localhost:1883";
    private static final int QoS = 2;
    private static final Logger LOG = LoggerFactory.getLogger(HouseManager.class);
    private final HouseEntity house;
    private final Solver solver = new Solver();
    private final TimelinesExecutor tl_exec = new TimelinesExecutor(solver, new Rational(1));
    private static final ScheduledExecutorService EXECUTOR = Executors
            .newScheduledThreadPool(Runtime.getRuntime().availableProcessors());
    private ScheduledFuture<?> scheduled_feature;
    private MqttClient mqtt_client;

    public HouseManager(final HouseEntity house) {
        this.house = house;
        this.tl_exec.addExecutorListener(this);
        try {
            mqtt_client = new MqttClient(BROKER_URL, "house-" + house.getId());

            final MqttConnectOptions connect_options = new MqttConnectOptions();
            connect_options.setCleanSession(true);

            mqtt_client.connect(connect_options);

            mqtt_client.subscribe(house.getId() + "/0/planner/in/plan", (topic, message) -> {
                // we read the problem..
                LOG.info("Reading the problem..");
                solver.read(new String(message.getPayload()));

                // we solve the problem..
                LOG.info("Solving the problem..");
                solver.solve();

                // we execute the solution..
                LOG.info("Starting execution..");
                scheduled_feature = EXECUTOR.scheduleAtFixedRate(() -> tl_exec.tick(), 0, 1000, TimeUnit.SECONDS);
            });

            mqtt_client.subscribe(house.getId() + "/0/planner/in/cant-start-yet", (topic, message) -> tl_exec
                    .dont_start_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));

            mqtt_client.subscribe(house.getId() + "/0/planner/in/cant-finish-yet", (topic, message) -> tl_exec
                    .dont_end_yet(App.MAPPER.readValue(new String(message.getPayload()), long[].class)));
        } catch (final MqttException ex) {
            LOG.error("Cannot create MQTT client..", ex);
        }
    }

    public Solver getSolver() {
        return solver;
    }

    @Override
    public void endingAtoms(final long[] atoms) {
        try {
            mqtt_client.publish(house.getId() + "/0/planner/out/finishing",
                    App.MAPPER.writeValueAsString(atoms).getBytes(), QoS, false);
        } catch (final JsonProcessingException | MqttException ex) {
            LOG.error("Cannot create MQTT message..", ex);
        }
    }

    @Override
    public void startingAtoms(final long[] atoms) {
        throw new UnsupportedOperationException("Not implemented yet..");
    }

    @Override
    public void tick(final Rational current_time) {
        try {
            if (((ArithItem) solver.get("horizon")).getValue().leq(current_time)) {
                LOG.info("Nothing more to be executed..");
                scheduled_feature.cancel(false);
            }
        } catch (NoSuchFieldException e) {
            e.printStackTrace();
        }
    }

    @SuppressWarnings("unused")
    private static final class Command {

        private final long id;
        private final String predicate;
        private final Map<String, Object> parameters = new HashMap<>();

        private Command(Atom atom) {
            this.id = atom.getSigma(); // TODO: not the right id..
            this.predicate = atom.getType().getName();
            atom.getExprs().entrySet().forEach(expr -> {
                switch (expr.getValue().getType().getName()) {
                case Solver.BOOL:
                    parameters.put(expr.getKey(), ((Item.BoolItem) expr.getValue()).getValue());
                case Solver.INT:
                case Solver.REAL:
                case Solver.TP:
                    parameters.put(expr.getKey(), ((Item.ArithItem) expr.getValue()).getValue());
                case Solver.STRING:
                    parameters.put(expr.getKey(), ((Item.StringItem) expr.getValue()).getValue());
                default:
                    String val = new String();
                    if (expr.getValue() instanceof EnumItem) {
                        if (((EnumItem) expr.getValue()).getVals().length == 1)
                            val += ((EnumItem) expr.getValue()).getVals()[0].getName();
                        else
                            val += Stream.of(((EnumItem) expr.getValue()).getVals()).map(itm -> itm.getName())
                                    .collect(Collectors.joining(", "));
                    } else
                        val += expr.getValue().getName();
                    parameters.put(expr.getKey(), val);
                }
            });
        }

        public long getId() {
            return id;
        }

        public String getPredicate() {
            return predicate;
        }

        public Map<String, Object> getParameters() {
            return Collections.unmodifiableMap(parameters);
        }
    }
}
