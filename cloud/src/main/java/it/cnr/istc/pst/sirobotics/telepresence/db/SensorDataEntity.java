package it.cnr.istc.pst.sirobotics.telepresence.db;

import java.util.Date;

import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.Temporal;
import javax.persistence.TemporalType;

@Entity
public class SensorDataEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long id;
    @Temporal(TemporalType.TIMESTAMP)
    private Date sensing_time;
    private String raw_data;

    public Long getId() {
        return id;
    }

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
