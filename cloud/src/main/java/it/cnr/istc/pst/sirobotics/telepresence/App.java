package it.cnr.istc.pst.sirobotics.telepresence;

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
import java.util.Set;
import java.util.stream.Collectors;

import javax.crypto.SecretKeyFactory;
import javax.crypto.spec.PBEKeySpec;
import javax.persistence.EntityManager;
import javax.persistence.EntityManagerFactory;
import javax.persistence.Persistence;

import com.fasterxml.jackson.databind.ObjectMapper;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.Javalin;
import io.javalin.core.security.Role;
import io.javalin.http.Context;
import io.javalin.http.Handler;
import io.javalin.http.UnauthorizedResponse;
import io.moquette.broker.Server;
import io.moquette.broker.config.ClasspathResourceLoader;
import io.moquette.broker.config.ResourceLoaderConfig;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class App {

    private static final Logger LOG = LoggerFactory.getLogger(App.class);
    static final ObjectMapper MAPPER = new ObjectMapper();
    static final EntityManagerFactory EMF = Persistence.createEntityManagerFactory("SI-Robotics_PU");
    private static final SecureRandom RAND = new SecureRandom();
    // private static final int ITERATIONS = 65536;
    private static final int ITERATIONS = 5;
    private static final int KEY_LENGTH = 512;
    private static final String ALGORITHM = "PBKDF2WithHmacSHA512";
    private static final Map<Long, HouseManager> MANAGERS = new HashMap<>();

    public static void main(String[] args) {
        final Server broker = new Server();
        try {
            broker.startServer(new ResourceLoaderConfig(new ClasspathResourceLoader()));
        } catch (IOException ex) {
            LOG.error("Cannot start SI-Robotics MQTT broker..", ex);
        }

        // Bind a shutdown hook
        Runtime.getRuntime().addShutdownHook(new Thread(() -> {
            LOG.info("Stopping SI-Robotics MQTT broker..");
            broker.stopServer();
            LOG.info("SI-Robotics MQTT broker stopped..");
        }));

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

                final List<HouseEntity> houses = em.createQuery("SELECT he FROM HouseEntity he", HouseEntity.class)
                        .getResultList();

                LOG.info("Loading {} houses..", houses.size());
                for (HouseEntity house : houses)
                    MANAGERS.put(house.getId(), new HouseManager(house));
            });
        });

        app.start();

        LOG.info("SI-Robotics server started! Press [CTRL+C] to stop..");
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
                return Collections.singleton(ExplRole.Guest);
            else
                return user_entity.getRoles().stream().map(r -> ExplRole.valueOf(r)).collect(Collectors.toSet());
        }
        return Collections.singleton(ExplRole.Guest);
    }

    public static String generateSalt() {
        byte[] salt = new byte[KEY_LENGTH];
        RAND.nextBytes(salt);
        return Base64.getEncoder().encodeToString(salt);
    }

    public static String hashPassword(String password, String salt) {
        char[] chars = password.toCharArray();
        byte[] bytes = salt.getBytes();

        PBEKeySpec spec = new PBEKeySpec(chars, bytes, ITERATIONS, KEY_LENGTH);

        Arrays.fill(chars, Character.MIN_VALUE);

        try {
            SecretKeyFactory fac = SecretKeyFactory.getInstance(ALGORITHM);
            byte[] securePassword = fac.generateSecret(spec).getEncoded();
            return Base64.getEncoder().encodeToString(securePassword);
        } catch (NoSuchAlgorithmException | InvalidKeySpecException ex) {
            return null;
        } finally {
            spec.clearPassword();
        }
    }

    enum ExplRole implements Role {
        Guest, User, Admin
    }
}
