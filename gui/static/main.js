let ws;
setup_ws();

function setup_ws() {
    ws = new WebSocket('ws://localhost:8080/solver');
    ws.onmessage = msg => { }
    ws.onclose = () => setTimeout(setup_ws, 1000);
}