package it.cnr.istc.pst.sirobotics.telepresence.db;

import javax.persistence.Entity;

@Entity
public class RobotTypeEntity extends DeviceTypeEntity {

    private String predicates;

    public String getPredicates() {
        return predicates;
    }

    public void setPredicates(String predicates) {
        this.predicates = predicates;
    }
}
