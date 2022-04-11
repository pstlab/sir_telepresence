package it.cnr.istc.sirobotics;

import android.util.Log;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;

public class WSConnection {

    private static final String TAG = "SI-Robotics";
    private static WSConnection instance;
    private final OkHttpClient client;
    private String host = "localhost", port = "8080", url = "ws://" + host + ":" + port + "/solver";
    private WebSocketListener listener;
    private Request request;
    private WebSocket ws;

    public static WSConnection getInstance() {
        if (instance == null) instance = new WSConnection();
        return instance;
    }

    private WSConnection() {
        client = new OkHttpClient();
    }

    public void setHost(String host) {
        this.host = host;
        this.url = url = "ws://" + host + ":" + port + "/solver";
    }

    public void setPort(String port) {
        this.port = port;
        this.url = url = "ws://" + host + ":" + port + "/solver";
    }

    public void setListener(WebSocketListener listener) {
        this.listener = listener;
    }

    public void connect() {
        Log.i(TAG, "Connecting to " + url);
        request = new Request.Builder().url(url).build();
        ws = client.newWebSocket(request, listener);
    }

    public void send(final String msg) {
        ws.send(msg);
    }
}
