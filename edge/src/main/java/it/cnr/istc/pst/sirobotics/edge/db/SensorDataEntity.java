package it.cnr.istc.pst.sirobotics.edge.db;

import java.util.Date;

import javax.persistence.Entity;
import javax.persistence.Id;
import javax.persistence.Temporal;
import javax.persistence.TemporalType;

@Entity
public class SensorDataEntity {

    @Id
    @Temporal(TemporalType.TIMESTAMP)
    private Date sensing_time;
    private String raw_data;

    public Date getSensingTime() {
        return sensing_time;
    }

    public void setSensingTime(Date sensing_time) {
        this.sensing_time = sensing_time;
    }

    public String getRawData() {
        return raw_data;
    }

    public void setRawData(String raw_data) {
        this.raw_data = raw_data;
    }
}
