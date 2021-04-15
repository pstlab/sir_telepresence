package it.cnr.istc.pst.sirobotics.telepresence;

import static io.javalin.apibuilder.ApiBuilder.delete;
import static io.javalin.apibuilder.ApiBuilder.get;
import static io.javalin.apibuilder.ApiBuilder.path;
import static io.javalin.apibuilder.ApiBuilder.post;
import static io.javalin.core.security.SecurityUtil.roles;

import java.io.IOException;
import java.security.NoSuchAlgorithmException;
import java.security.SecureRandom;
import java.security.spec.InvalidKeySpecException;
import java.util.Arrays;
import java.util.Base64;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.stream.Collectors;

import javax.crypto.SecretKeyFactory;
import javax.crypto.spec.PBEKeySpec;
import javax.persistence.EntityManager;
import javax.persistence.EntityManagerFactory;
import javax.persistence.Persistence;

import com.fasterxml.jackson.annotation.JsonAutoDetect.Visibility;
import com.fasterxml.jackson.annotation.PropertyAccessor;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.eclipse.paho.client.mqttv3.IMqttMessageListener;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.core.security.Role;
import io.javalin.http.Context;
import io.javalin.http.Handler;
import io.javalin.http.UnauthorizedResponse;
import io.javalin.plugin.json.JavalinJackson;
import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.sirobotics.telepresence.db.DeviceTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final Properties PROPERTIES = new Properties();
    static final int QoS = 2;
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final EntityManagerFactory EMF = Persistence.createEntityManagerFactory("SI-Robotics_PU");
    private static final SecureRandom RAND = new SecureRandom();
    // private static final int ITERATIONS = 65536;
    private static final int ITERATIONS = 5;
    private static final int KEY_LENGTH = 512;
    private static final String ALGORITHM = "PBKDF2WithHmacSHA512";
    static final Map<Long, HouseManager> MANAGERS = new HashMap<>();
    static MqttClient MQTT_CLIENT;
    static RasaClient NLU_CLIENT;

    public static void main(final String[] args) {
        LOG.info("Current library path: {}", System.getProperty("java.library.path"));

        MAPPER.setVisibility(PropertyAccessor.FIELD, Visibility.ANY);
        JavalinJackson.configure(MAPPER);

        try {
            PROPERTIES.load(App.class.getClassLoader().getResourceAsStream("config.properties"));
        } catch (final IOException ex) {
            LOG.error("Cannot load config file..", ex);
        }

        try {
            LOG.info("Creating the SI-Robotics cloud MQTT client..");
            final String server_uri = "tcp://" + PROPERTIES.getProperty("mqtt_host", "localhost") + ':'
                    + PROPERTIES.getProperty("mqtt_port", "1883");
            MQTT_CLIENT = new MqttClient(server_uri, "SI-Robotics cloud", new MemoryPersistence());

            LOG.info("Connecting the SI-Robotics cloud MQTT client to the broker..");
            final MqttConnectOptions connect_options = new MqttConnectOptions();
            connect_options.setCleanSession(true);
            connect_options.setWill("cloud/online", "false".getBytes(), QoS, true);
            MQTT_CLIENT.connect(connect_options);
            MQTT_CLIENT.publish("cloud/online", "true".getBytes(), QoS, true);

            LOG.info("Subscribing the SI-Robotics cloud MQTT client to all the topics..");
            MQTT_CLIENT.subscribe("#", new IMqttMessageListener() {

                @Override
                public void messageArrived(final String topic, final MqttMessage message) throws Exception {
                    LOG.info("Topic: {}", topic);
                    LOG.info("Payload: {}", new String(message.getPayload()));
                }
            });
        } catch (final MqttException ex) {
            LOG.error("Cannot create MQTT client..", ex);
        }

        LOG.info("Connecting to the NLU provider..");
        NLU_CLIENT = new RasaClient();
        try {
            JsonNode version = NLU_CLIENT.version();
            LOG.info("NLU Provider: {}", version);
        } catch (Exception ex) {
            LOG.error("Cannot connect to the NLU provider..", ex);
        }

        final Javalin app = Javalin.create(config -> {
            config.addStaticFiles("/public");
            config.accessManager((final Handler handler, final Context ctx, final Set<Role> permittedRoles) -> {
                if (getRoles(ctx).stream().anyMatch(role -> permittedRoles.contains(role)))
                    handler.handle(ctx);
                else
                    throw new UnauthorizedResponse();
            });
            config.enableCorsForAllOrigins();
        });

        app.events(event -> {
            event.serverStarting(() -> LOG.info("Starting SI-Robotics web server.."));
            event.serverStarted(() -> {
                LOG.info("SI-Robotics web server is running..");
                final EntityManager em = App.EMF.createEntityManager();

                final List<UserEntity> users = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                        .getResultList();

                LOG.info("Loading {} users..", users.size());
                if (users.isEmpty()) {
                    LOG.info("Creating new admin user..");
                    final UserEntity admin = new UserEntity();
                    admin.setEmail("admin");
                    final String salt = generateSalt();
                    admin.setSalt(salt);
                    admin.setPassword(hashPassword("admin", salt));
                    admin.setFirstName("Admin");
                    admin.setLastName("Admin");
                    admin.addRole(SIRRole.Admin.name());

                    em.getTransaction().begin();
                    em.persist(admin);
                    em.getTransaction().commit();
                }

                final List<DeviceTypeEntity> device_types = em
                        .createQuery("SELECT dt FROM DeviceTypeEntity dt", DeviceTypeEntity.class).getResultList();

                LOG.info("Loading {} device types..", device_types.size());
                if (device_types.isEmpty()) {
                    LOG.info("Creating sample robot type..");
                    final DeviceTypeEntity ohmni_type = new RobotTypeEntity();
                    ohmni_type.setName("Ohmni Robot");
                    ohmni_type.setDescription(
                            "Un robot di telepresenza che trasforma il modo in cui le persone si connettono.");

                    em.getTransaction().begin();
                    em.persist(ohmni_type);
                    em.getTransaction().commit();
                }

                final List<HouseEntity> houses = em.createQuery("SELECT he FROM HouseEntity he", HouseEntity.class)
                        .getResultList();

                LOG.info("Loading {} houses..", houses.size());
                for (final HouseEntity house : houses)
                    MANAGERS.put(house.getId(), new HouseManager(house));

                em.close();
            });
        });

        // we create the routes..
        app.routes(() -> {
            post("login", UserController::login, roles(SIRRole.Guest, SIRRole.Admin));
            path("user", () -> {
                post(UserController::createUser, roles(SIRRole.Guest, SIRRole.Admin));
                path(":id", () -> {
                    get(UserController::getUser, roles(SIRRole.Admin, SIRRole.User));
                    post(UserController::updateUser, roles(SIRRole.Admin, SIRRole.User));
                    delete(UserController::deleteUser, roles(SIRRole.Admin, SIRRole.User));
                });
            });
            path("users", () -> get(UserController::getAllUsers, roles(SIRRole.Admin)));
            path("house", () -> {
                post(HouseController::createHouse, roles(SIRRole.Admin));
                path(":id", () -> {
                    get(HouseController::getHouse, roles(SIRRole.Admin, SIRRole.User));
                });
            });
            path("house_params", () -> get(HouseController::houseParams, roles(SIRRole.Admin)));
            path("assign", () -> post(HouseController::assignUser, roles(SIRRole.Admin)));
            path("unassign", () -> post(HouseController::unassignUser, roles(SIRRole.Admin)));
            path("device", () -> post(HouseController::createDevice, roles(SIRRole.Admin)));
            path("houses", () -> get(HouseController::getAllHouses, roles(SIRRole.Admin)));
            path("sensor_type", () -> post(HouseController::createSensorType, roles(SIRRole.Admin)));
            path("robot_type", () -> post(HouseController::createRobotType, roles(SIRRole.Admin)));
            path("device_types", () -> get(HouseController::getAllDeviceTypes, roles(SIRRole.Admin)));
        });

        app.ws("/communication", ws -> {
            ws.onConnect(ctx -> new_connection(ctx));
            ws.onClose(ctx -> lost_connection(ctx));
            ws.onMessage(ctx -> new_message(ctx));
        }, roles(SIRRole.Admin, SIRRole.User));

        app.start();

        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            app.stop();
            EMF.close();
        }));

        LOG.info("SI-Robotics server started! Press [CTRL+C] to stop..");
    }

    private static synchronized void new_connection(final WsContext ctx) {
        final Long id = Long.valueOf(ctx.queryParam("id"));
        LOG.info("User #{} connected..", id);
        UserController.ONLINE.put(id, ctx);
    }

    private static synchronized void lost_connection(final WsContext ctx) {
        final Long id = Long.valueOf(ctx.queryParam("id"));
        LOG.info("User #{} disconnected..", id);
        UserController.ONLINE.remove(id);
    }

    private static synchronized void new_message(final WsContext ctx) {
        LOG.info("Received message {}..", ctx);
    }

    static Set<Role> getRoles(final Context ctx) {
        final String auth_head = ctx.header("Authorization");
        Long id = null;
        if (auth_head != null)
            id = Long.valueOf(auth_head.replace("Basic ", ""));
        final String ws_protocol_head = ctx.header("Sec-WebSocket-Protocol");
        if (ws_protocol_head != null)
            id = Long.valueOf(ctx.queryParam("id"));
        if (id != null) {
            final EntityManager em = App.EMF.createEntityManager();
            final UserEntity user_entity = em.find(UserEntity.class, id);
            em.close();
            if (user_entity == null)
                return Collections.singleton(SIRRole.Guest);
            else
                return user_entity.getRoles().stream().map(r -> SIRRole.valueOf(r)).collect(Collectors.toSet());
        }
        return Collections.singleton(SIRRole.Guest);
    }

    public static String generateSalt() {
        final byte[] salt = new byte[KEY_LENGTH];
        RAND.nextBytes(salt);
        return Base64.getEncoder().encodeToString(salt);
    }

    public static String hashPassword(final String password, final String salt) {
        final char[] chars = password.toCharArray();
        final byte[] bytes = salt.getBytes();

        final PBEKeySpec spec = new PBEKeySpec(chars, bytes, ITERATIONS, KEY_LENGTH);

        Arrays.fill(chars, Character.MIN_VALUE);

        try {
            final SecretKeyFactory fac = SecretKeyFactory.getInstance(ALGORITHM);
            final byte[] securePassword = fac.generateSecret(spec).getEncoded();
            return Base64.getEncoder().encodeToString(securePassword);
        } catch (NoSuchAlgorithmException | InvalidKeySpecException ex) {
            return null;
        } finally {
            spec.clearPassword();
        }
    }

    enum SIRRole implements Role {
        Guest, User, Admin
    }
}
