package it.cnr.istc.pst.sirobotics.telepresence.db;

import javax.persistence.Entity;

@Entity
public class RobotTypeEntity extends DeviceTypeEntity {

    private String configuration;

    public String getConfiguration() {
        return configuration;
    }

    public void setConfiguration(String configuration) {
        this.configuration = configuration;
    }
}
