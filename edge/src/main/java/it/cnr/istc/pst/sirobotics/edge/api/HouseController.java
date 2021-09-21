package it.cnr.istc.pst.sirobotics.edge.api;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import it.cnr.istc.pst.sirobotics.edge.api.Device.Robot;
import it.cnr.istc.pst.sirobotics.edge.api.Device.Sensor;
import it.cnr.istc.pst.sirobotics.edge.api.Device.Sensor.Data;
import it.cnr.istc.pst.sirobotics.edge.db.DeviceEntity;
import it.cnr.istc.pst.sirobotics.edge.db.DeviceRepository;
import it.cnr.istc.pst.sirobotics.edge.db.DeviceTypeEntity;
import it.cnr.istc.pst.sirobotics.edge.db.DeviceTypeRepository;
import it.cnr.istc.pst.sirobotics.edge.db.RobotEntity;
import it.cnr.istc.pst.sirobotics.edge.db.RobotTypeEntity;
import it.cnr.istc.pst.sirobotics.edge.db.SensorEntity;
import it.cnr.istc.pst.sirobotics.edge.db.SensorTypeEntity;

@RestController
public class HouseController {

    @Autowired
    DeviceTypeRepository device_type_repository;
    @Autowired
    DeviceRepository device_repository;

    @GetMapping("/device_types")
    public List<DeviceType> get_device_types() {
        return device_type_repository.findAll().stream().map(dev_type -> toDeviceType(dev_type))
                .collect(Collectors.toList());
    }

    @GetMapping("/devices")
    public List<Device> get_devices() {
        return device_repository.findAll().stream().map(dev -> toDevice(dev, false)).collect(Collectors.toList());
    }

    @PostMapping("/devices")
    Device new_device(@RequestParam(value = "name", required = true) final String name,
            @RequestParam(value = "name") final String description,
            @RequestParam(value = "type_id", required = true) final Long type_id) {
        final Optional<DeviceTypeEntity> opt_dev_type = device_type_repository.findById(type_id);

        if (opt_dev_type.isPresent()) {
            final DeviceTypeEntity dev_type = opt_dev_type.get();
            DeviceEntity dev = null;
            if (dev_type instanceof SensorTypeEntity)
                dev = new SensorEntity();
            else if (dev_type instanceof RobotTypeEntity)
                dev = new RobotEntity();
            else
                throw new UnsupportedOperationException();

            dev.setName(name);
            dev.setDescription(description);
            dev.setType(dev_type);

            return toDevice(device_repository.save(dev), false);
        } else
            throw new DeviceTypeNotFoundException(type_id);
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
