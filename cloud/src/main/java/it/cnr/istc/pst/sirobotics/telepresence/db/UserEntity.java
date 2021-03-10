package it.cnr.istc.pst.sirobotics.telepresence.db;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import javax.persistence.Column;
import javax.persistence.ElementCollection;
import javax.persistence.Entity;
import javax.persistence.GeneratedValue;
import javax.persistence.GenerationType;
import javax.persistence.Id;
import javax.persistence.ManyToMany;

@Entity
public class UserEntity {

    @Id
    @GeneratedValue(strategy = GenerationType.AUTO)
    private Long id;
    @Column(nullable = false)
    private String email;
    @Column(nullable = false, length = 684)
    private String salt;
    @Column(nullable = false)
    private String password;
    private String first_name;
    private String last_name;
    @ManyToMany
    private Collection<HouseEntity> houses = new ArrayList<>();
    @ElementCollection
    private final Collection<String> roles = new ArrayList<>();

    public Long getId() {
        return id;
    }

    public String getEmail() {
        return email;
    }

    public void setEmail(String email) {
        this.email = email;
    }

    public String getSalt() {
        return salt;
    }

    public void setSalt(String salt) {
        this.salt = salt;
    }

    public String getPassword() {
        return password;
    }

    public void setPassword(String password) {
        this.password = password;
    }

    public String getFirstName() {
        return first_name;
    }

    public void setFirstName(String first_name) {
        this.first_name = first_name;
    }

    public String getLastName() {
        return last_name;
    }

    public void setLastName(String last_name) {
        this.last_name = last_name;
    }

    public Collection<String> getRoles() {
        return Collections.unmodifiableCollection(roles);
    }

    public void addRole(String role) {
        roles.add(role);
    }

    public void removeRole(String role) {
        roles.remove(role);
    }

    public Collection<HouseEntity> getHouses() {
        return Collections.unmodifiableCollection(houses);
    }

    public void addHouse(HouseEntity house) {
        houses.add(house);
    }

    public void removeHouse(HouseEntity house) {
        houses.remove(house);
    }
}
