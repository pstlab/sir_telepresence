package it.cnr.istc.pst.sirobotics.edge.api;

class DeviceTypeNotFoundException extends RuntimeException {

    DeviceTypeNotFoundException(Long id) {
        super("Could not find device type " + id);
    }
}
