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
    @ManyToMany
    private Collection<UserEntity> users = new ArrayList<>();
    @OneToMany
    private Collection<DeviceEntity> devices = new ArrayList<>();

    public Long getId() {
        return id;
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

    public Collection<UserEntity> getUsers() {
        return Collections.unmodifiableCollection(users);
    }

    public void addUser(UserEntity user) {
        users.add(user);
    }

    public void removeUser(UserEntity user) {
        users.remove(user);
    }
}
