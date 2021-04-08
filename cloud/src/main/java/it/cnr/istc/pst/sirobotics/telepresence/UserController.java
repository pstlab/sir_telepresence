package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javax.persistence.EntityManager;
import javax.persistence.NoResultException;
import javax.persistence.TypedQuery;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.http.ConflictResponse;
import io.javalin.http.Context;
import io.javalin.http.ForbiddenResponse;
import io.javalin.http.NotFoundResponse;
import io.javalin.websocket.WsContext;
import it.cnr.istc.pst.sirobotics.telepresence.App.SIRRole;
import it.cnr.istc.pst.sirobotics.telepresence.api.User;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class UserController {

    static final Logger LOG = LoggerFactory.getLogger(UserController.class);
    /**
     * For each user id, a boolean indicating whether the user is online.
     */
    static final Map<Long, WsContext> ONLINE = new HashMap<>();

    /**
     * Given an email and a password, returns the user corresponding to the email if
     * the password corresponds to the stored one.
     * 
     * @param ctx
     */
    static synchronized void login(final Context ctx) {
        final String email = ctx.formParam("email");
        final String password = ctx.formParam("password");
        LOG.info("user {} is logging in..", email);

        final EntityManager em = App.EMF.createEntityManager();
        final TypedQuery<UserEntity> query = em.createQuery("SELECT u FROM UserEntity u WHERE u.email = :email",
                UserEntity.class);
        query.setParameter("email", email);
        try {
            final UserEntity user_entity = query.getSingleResult();
            if (!App.hashPassword(password, user_entity.getSalt()).equals(user_entity.getPassword()))
                throw new ForbiddenResponse();

            ctx.json(toUser(user_entity));
        } catch (final NoResultException e) {
            throw new NotFoundResponse();
        }
        em.close();
    }

    /**
     * Returns all the stored users.
     * 
     * @param ctx
     */
    static synchronized void getAllUsers(final Context ctx) {
        LOG.info("retrieving all users..");
        final EntityManager em = App.EMF.createEntityManager();
        final List<UserEntity> user_entities = em.createQuery("SELECT ue FROM UserEntity ue", UserEntity.class)
                .getResultList();

        ctx.json(user_entities.stream().map(user -> toUser(user)).collect(Collectors.toList()));
        em.close();
    }

    /**
     * Creates a new user and stores it into the database.
     * 
     * @param ctx
     */
    static synchronized void createUser(final Context ctx) {
        final String email = ctx.formParam("email");
        final String salt = App.generateSalt();
        final String password = App.hashPassword(ctx.formParam("password"), salt);
        final String first_name = ctx.formParam("first_name");
        final String last_name = ctx.formParam("last_name");
        LOG.info("creating new user {}..", email);
        final EntityManager em = App.EMF.createEntityManager();

        final UserEntity user_entity = new UserEntity();
        user_entity.setEmail(email);
        user_entity.setSalt(salt);
        user_entity.setPassword(password);
        user_entity.setFirstName(first_name);
        user_entity.setLastName(last_name);
        user_entity.addRole(SIRRole.User.name());

        try {
            em.getTransaction().begin();
            em.persist(user_entity);
            em.getTransaction().commit();
        } catch (final Exception ex) {
            throw new ConflictResponse();
        }

        ctx.status(201);
        em.close();
    }

    /**
     * Returns, if exists, a stored user having the given id.
     * 
     * @param ctx
     */
    static synchronized void getUser(final Context ctx) {
        final long user_id = Long.valueOf(ctx.queryParam("id"));
        LOG.info("retrieving user #{}..", user_id);
        final EntityManager em = App.EMF.createEntityManager();
        final UserEntity user_entity = em.find(UserEntity.class, user_id);
        if (user_entity == null)
            throw new NotFoundResponse();

        ctx.json(toUser(user_entity));
        em.close();
    }

    static synchronized void updateUser(final Context ctx) {
        final long user_id = Long.valueOf(ctx.pathParam("id"));
        LOG.info("updating user #{}..", user_id);
        final User user = ctx.bodyAsClass(User.class);

        final EntityManager em = App.EMF.createEntityManager();
        final UserEntity user_entity = em.find(UserEntity.class, user_id);
        if (user_entity == null)
            throw new NotFoundResponse();

        em.getTransaction().begin();
        user_entity.setFirstName(user.getFirstName());
        user_entity.setLastName(user.getLastName());
        em.getTransaction().commit();

        ctx.status(204);
        em.close();
    }

    static synchronized void deleteUser(final Context ctx) {
        final long user_id = Long.valueOf(ctx.pathParam("id"));
        LOG.info("deleting user #{}..", user_id);
        final EntityManager em = App.EMF.createEntityManager();
        final UserEntity user_entity = em.find(UserEntity.class, user_id);
        if (user_entity == null)
            throw new NotFoundResponse();

        em.getTransaction().begin();
        em.remove(user_entity);
        em.getTransaction().commit();

        ctx.status(204);
        em.close();
    }

    static User toUser(final UserEntity entity) {
        final boolean online = ONLINE.containsKey(entity.getId());

        return new User(entity.getId(), entity.getEmail(), entity.getFirstName(), entity.getLastName(),
                entity.getRoles(), online);
    }
}
