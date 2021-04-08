package it.cnr.istc.pst.sirobotics.telepresence.db;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.persistence.Entity;
import javax.persistence.OneToMany;

@Entity
public class RobotEntity extends DeviceEntity {

    @OneToMany
    private Collection<DeviceEntity> devices = new ArrayList<>();

    public Collection<DeviceEntity> getDevices() {
        return Collections.unmodifiableCollection(devices);
    }

    public void addDevice(DeviceEntity device) {
        devices.add(device);
    }

    public void removeDevice(DeviceEntity device) {
        devices.remove(device);
    }
}
