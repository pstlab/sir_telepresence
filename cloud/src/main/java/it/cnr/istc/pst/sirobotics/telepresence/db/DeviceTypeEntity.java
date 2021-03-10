package it.cnr.istc.pst.sirobotics.telepresence.db;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.OneToMany;

@Entity
public class DeviceTypeEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long id;
    private String description;
    @OneToMany
    private Collection<DeviceEntity> devices = new ArrayList<>();

    public Long getId() {
        return id;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(String description) {
        this.description = description;
    }

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
