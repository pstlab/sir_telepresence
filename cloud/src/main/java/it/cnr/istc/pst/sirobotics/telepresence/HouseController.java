package it.cnr.istc.pst.sirobotics.telepresence;

import java.util.List;
import java.util.stream.Collectors;

import javax.persistence.EntityManager;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import io.javalin.http.ConflictResponse;
import io.javalin.http.Context;
import io.javalin.http.NotFoundResponse;
import it.cnr.istc.pst.sirobotics.telepresence.api.Device;
import it.cnr.istc.pst.sirobotics.telepresence.api.Device.Robot;
import it.cnr.istc.pst.sirobotics.telepresence.api.Device.Sensor;
import it.cnr.istc.pst.sirobotics.telepresence.api.Device.Sensor.Data;
import it.cnr.istc.pst.sirobotics.telepresence.api.DeviceType;
import it.cnr.istc.pst.sirobotics.telepresence.api.House;
import it.cnr.istc.pst.sirobotics.telepresence.api.User;
import it.cnr.istc.pst.sirobotics.telepresence.db.DeviceEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.DeviceTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.HouseEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.RobotTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.SensorTypeEntity;
import it.cnr.istc.pst.sirobotics.telepresence.db.UserEntity;

public class HouseController {

    static final Logger LOG = LoggerFactory.getLogger(UserController.class);

    /**
     * Returns all the stored houses.
     * 
     * @param ctx
     */
    static synchronized void getAllHouses(final Context ctx) {
        LOG.info("retrieving all houses..");
        final EntityManager em = App.EMF.createEntityManager();
        final List<HouseEntity> house_entities = em.createQuery("SELECT he FROM HouseEntity he", HouseEntity.class)
                .getResultList();

        ctx.json(house_entities.stream().map(house -> toHouse(house, false)).collect(Collectors.toList()));
        em.close();
    }

    /**
     * Creates a new house and stores it into the database.
     * 
     * @param ctx
     */
    static synchronized void createHouse(final Context ctx) {
        final String name = ctx.formParam("name");
        final String description = ctx.formParam("description");
        LOG.info("creating new house {}..", name);
        final EntityManager em = App.EMF.createEntityManager();

        final HouseEntity house_entity = new HouseEntity();
        house_entity.setName(name);
        house_entity.setDescription(description);

        try {
            em.getTransaction().begin();
            em.persist(house_entity);
            em.getTransaction().commit();
        } catch (final Exception ex) {
            throw new ConflictResponse();
        }

        App.MANAGERS.put(house_entity.getId(), new HouseManager(house_entity));

        ctx.json(toHouse(house_entity, false));
        em.close();
    }

    /**
     * Returns, if exists, a stored house having the given id.
     * 
     * @param ctx
     */
    static synchronized void getHouse(final Context ctx) {
        final long house_id = Long.valueOf(ctx.queryParam("id"));
        LOG.info("retrieving house #{}..", house_id);
        final EntityManager em = App.EMF.createEntityManager();
        final HouseEntity house_entity = em.find(HouseEntity.class, house_id);
        if (house_entity == null)
            throw new NotFoundResponse();

        ctx.json(toHouse(house_entity, true));
        em.close();
    }

    /**
     * Returns all the stored sensor types.
     * 
     * @param ctx
     */
    static synchronized void getAllDeviceTypes(final Context ctx) {
        LOG.info("retrieving all sensor types..");
        final EntityManager em = App.EMF.createEntityManager();
        final List<DeviceTypeEntity> device_type_entities = em
                .createQuery("SELECT dt FROM DeviceTypeEntity dt", DeviceTypeEntity.class).getResultList();

        ctx.json(
                device_type_entities.stream().map(device_type -> toDeviceType(device_type)).toArray(DeviceType[]::new));
        em.close();
    }

    /**
     * Creates a new sensor type and stores it into the database.
     * 
     * @param ctx
     */
    static synchronized void createSensorType(final Context ctx) {
        final String name = ctx.formParam("name");
        final String description = ctx.formParam("description");
        LOG.info("creating new sensor type {}..", name);

        final EntityManager em = App.EMF.createEntityManager();

        final SensorTypeEntity sensor_type_entity = new SensorTypeEntity();
        sensor_type_entity.setName(name);
        sensor_type_entity.setDescription(description);

        try {
            em.getTransaction().begin();
            em.persist(sensor_type_entity);
            em.getTransaction().commit();
        } catch (final Exception ex) {
            throw new ConflictResponse();
        }

        ctx.json(toDeviceType(sensor_type_entity));
        em.close();
    }

    /**
     * Creates a new robot type and stores it into the database.
     * 
     * @param ctx
     */
    static synchronized void createRobotType(final Context ctx) {
        final String name = ctx.formParam("name");
        final String description = ctx.formParam("description");
        LOG.info("creating new device type {}..", name);

        final EntityManager em = App.EMF.createEntityManager();

        final RobotTypeEntity robot_type_entity = new RobotTypeEntity();
        robot_type_entity.setName(name);
        robot_type_entity.setDescription(description);

        try {
            em.getTransaction().begin();
            em.persist(robot_type_entity);
            em.getTransaction().commit();
        } catch (final Exception ex) {
            throw new ConflictResponse();
        }

        ctx.json(toDeviceType(robot_type_entity));
        em.close();
    }

    /**
     * Creates a new device and stores it into the database.
     * 
     * @param ctx
     */
    static synchronized void createDevice(final Context ctx) {
        final long house_id = Long.valueOf(ctx.formParam("house_id"));
        final String name = ctx.formParam("name");
        final String description = ctx.formParam("description");
        final long type_id = Long.valueOf(ctx.formParam("type_id"));
        LOG.info("creating new device type {}..", name);

        final EntityManager em = App.EMF.createEntityManager();
        final HouseEntity house_entity = em.find(HouseEntity.class, house_id);
        if (house_entity == null)
            throw new NotFoundResponse();
        final DeviceTypeEntity device_type_entity = em.find(DeviceTypeEntity.class, type_id);
        if (device_type_entity == null)
            throw new NotFoundResponse();

        DeviceEntity device_entity = null;
        if (device_type_entity instanceof SensorTypeEntity)
            device_entity = new SensorEntity();
        else if (device_type_entity instanceof RobotTypeEntity)
            device_entity = new RobotEntity();
        else
            throw new UnsupportedOperationException();

        device_entity.setName(name);
        device_entity.setDescription(description);
        device_entity.setType(device_type_entity);

        try {
            em.getTransaction().begin();
            house_entity.addDevice(device_entity);
            em.persist(device_entity);
            em.getTransaction().commit();
        } catch (final Exception ex) {
            throw new ConflictResponse();
        }

        ctx.json(toDevice(device_entity, false));
        em.close();
    }

    static void assignUser(final Context ctx) {
        final long user_id = Long.valueOf(ctx.pathParam("user_id"));
        final long house_id = Long.valueOf(ctx.pathParam("house_id"));
        LOG.info("assigning user #{} to house #{}..", user_id, house_id);
        final EntityManager em = App.EMF.createEntityManager();
        final UserEntity user_entity = em.find(UserEntity.class, user_id);
        if (user_entity == null)
            throw new NotFoundResponse();
        final HouseEntity house_entity = em.find(HouseEntity.class, house_id);
        if (house_entity == null)
            throw new NotFoundResponse();

        em.getTransaction().begin();
        house_entity.addUser(user_entity);
        user_entity.addHouse(house_entity);
        em.getTransaction().commit();

        ctx.status(204);
        em.close();
    }

    static void unassignUser(final Context ctx) {
        final long user_id = Long.valueOf(ctx.pathParam("user_id"));
        final long house_id = Long.valueOf(ctx.pathParam("house_id"));
        LOG.info("unassigning user #{} from house #{}..", user_id, house_id);
        final EntityManager em = App.EMF.createEntityManager();
        final UserEntity user_entity = em.find(UserEntity.class, user_id);
        if (user_entity == null)
            throw new NotFoundResponse();
        final HouseEntity house_entity = em.find(HouseEntity.class, house_id);
        if (house_entity == null)
            throw new NotFoundResponse();

        em.getTransaction().begin();
        house_entity.removeUser(user_entity);
        user_entity.removeHouse(house_entity);
        em.getTransaction().commit();

        ctx.status(204);
        em.close();
    }

    static House toHouse(final HouseEntity entity, final boolean include_data) {
        return new House(entity.getId(), entity.getName(), entity.getDescription(),
                entity.getDevices().stream().map(device -> toDevice(device, include_data)).toArray(Device[]::new),
                entity.getUsers().stream().map(user -> UserController.toUser(user)).toArray(User[]::new));
    }

    static DeviceType toDeviceType(final DeviceTypeEntity entity) {
        if (entity instanceof SensorTypeEntity)
            return new DeviceType.SensorType(entity.getId(), entity.getName(), entity.getDescription());
        else if (entity instanceof RobotTypeEntity)
            return new DeviceType.RobotType(entity.getId(), entity.getName(), entity.getDescription());
        else
            throw new UnsupportedOperationException();
    }

    static Device toDevice(final DeviceEntity entity, final boolean include_data) {
        if (entity instanceof RobotEntity)
            return toRobot((RobotEntity) entity, include_data);
        else if (entity instanceof SensorEntity)
            return toSensor((SensorEntity) entity, include_data);
        throw new UnsupportedOperationException();
    }

    static Robot toRobot(final RobotEntity entity, final boolean include_data) {
        return new Robot(entity.getId(), entity.getName(), entity.getDescription(), toDeviceType(entity.getType()),
                entity.getDevices().stream().map(device -> toDevice(device, include_data)).toArray(Device[]::new));
    }

    static Sensor toSensor(final SensorEntity entity, final boolean include_data) {
        return new Sensor(entity.getId(), entity.getName(), entity.getDescription(), toDeviceType(entity.getType()),
                include_data ? entity.getData().stream()
                        .map(data -> new Data(data.getSensingTime().getTime(), data.getRawData())).toArray(Data[]::new)
                        : null);
    }
}
