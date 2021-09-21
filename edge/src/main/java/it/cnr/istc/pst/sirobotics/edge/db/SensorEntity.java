package it.cnr.istc.pst.sirobotics.edge.db;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.persistence.Entity;
import javax.persistence.OneToMany;

@Entity
public class SensorEntity extends DeviceEntity {

    private String position;
    @OneToMany
    private Collection<SensorDataEntity> sensor_data = new ArrayList<>();

    public String getPosition() {
        return position;
    }

    public void setPosition(String position) {
        this.position = position;
    }

    public Collection<SensorDataEntity> getData() {
        return Collections.unmodifiableCollection(sensor_data);
    }

    public void addData(SensorDataEntity data) {
        sensor_data.add(data);
    }

    public void removeData(SensorDataEntity data) {
        sensor_data.remove(data);
    }
}
