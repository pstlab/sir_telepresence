package it.cnr.istc.pst.sirobotics.telepresence.db;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.ManyToMany;
import javax.persistence.OneToMany;

@Entity
public class HouseEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long id;
    private String name, description;
    @ManyToMany
    private final Collection<UserEntity> users = new ArrayList<>();
    @OneToMany
    private final Collection<DeviceEntity> devices = new ArrayList<>();

    public Long getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public void setName(final String name) {
        this.name = name;
    }

    public String getDescription() {
        return description;
    }

    public void setDescription(final String description) {
        this.description = description;
    }

    public Collection<DeviceEntity> getDevices() {
        return Collections.unmodifiableCollection(devices);
    }

    public void addDevice(final DeviceEntity device) {
        devices.add(device);
    }

    public void removeDevice(final DeviceEntity device) {
        devices.remove(device);
    }

    public Collection<UserEntity> getUsers() {
        return Collections.unmodifiableCollection(users);
    }

    public void addUser(final UserEntity user) {
        users.add(user);
    }

    public void removeUser(final UserEntity user) {
        users.remove(user);
    }
}
