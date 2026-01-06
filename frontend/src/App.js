import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, useMap, Polyline, Polygon } from 'react-leaflet';
import axios from 'axios';
import io from 'socket.io-client';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import './App.css';

// --- Icons ---
const scoutIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-blue.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const deliveryIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const survivorIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-orange.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});
const detectionIcon = new L.Icon({
    iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-green.png',
    iconSize: [25, 41], iconAnchor: [12, 41], popupAnchor: [1, -34],
});

// --- Config ---
const IP_ADDRESS = "100.125.45.22";
const socket = io(`http://${IP_ADDRESS}:5000`);

// --- Helpers ---
function isPointInPoly(pt, vs) {
    var x = pt.lat, y = pt.lng;
    var inside = false;
    for (var i = 0, j = vs.length - 1; i < vs.length; j = i++) {
        var xi = vs[i].lat, yi = vs[i].lng;
        var xj = vs[j].lat, yj = vs[j].lng;
        var intersect = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

function generateGrid(polyPoints) {
    if (polyPoints.length < 3) return [];
    let minLat = 90, maxLat = -90, minLon = 180, maxLon = -180;
    polyPoints.forEach(p => {
        if (p.lat < minLat) minLat = p.lat;
        if (p.lat > maxLat) maxLat = p.lat;
        if (p.lng < minLon) minLon = p.lng;
        if (p.lng > maxLon) maxLon = p.lng;
    });

    const step = 0.00015; // ~15-20m
    let grid = [];
    let latSteps = Math.ceil((maxLat - minLat) / step);
    let lonSteps = Math.ceil((maxLon - minLon) / step);

    for (let i = 0; i <= latSteps; i++) {
        let currentLat = minLat + (i * step);
        let row = [];
        for (let j = 0; j <= lonSteps; j++) {
            let currentLon = minLon + (j * step);
            if (isPointInPoly({ lat: currentLat, lng: currentLon }, polyPoints)) {
                row.push({ lat: currentLat, lng: currentLon });
            }
        }
        if (i % 2 === 1) row.reverse(); // Zig-zag
        grid.push(...row);
    }
    return grid;
}

// --- UI Components ---
const Card = ({ children, title, className = "" }) => (
    <div className={`bg-white p-4 rounded-lg shadow-sm border border-slate-200 ${className}`}>
        {title && <h3 className="text-xs font-bold text-slate-400 uppercase tracking-wider mb-3">{title}</h3>}
        {children}
    </div>
);

const Button = ({ children, onClick, variant = "primary", className = "" }) => {
    const variants = {
        primary: "bg-slate-900 text-white hover:bg-slate-800",
        secondary: "bg-white text-slate-700 border border-slate-300 hover:bg-slate-50",
        danger: "bg-red-500 text-white hover:bg-red-600",
        success: "bg-green-600 text-white hover:bg-green-700",
        warning: "bg-orange-500 text-white hover:bg-orange-600"
    };
    return (
        <button
            onClick={onClick}
            className={`px-4 py-2 rounded-md text-sm font-medium transition-colors duration-200 ${variants[variant]} ${className}`}
        >
            {children}
        </button>
    );
};

const RcSlider = ({ label, val, setVal }) => (
    <div className="flex items-center gap-2">
        <span className="text-[10px] w-12 font-mono text-slate-500">{label}</span>
        <input
            type="range" min="1000" max="2000" step="10"
            value={val} onChange={(e) => setVal(e.target.value)}
            className="w-full h-1 bg-slate-200 rounded-lg appearance-none cursor-pointer"
        />
        <span className="text-[10px] w-8 font-mono">{val}</span>
    </div>
);

// --- Map Components ---
function MapClicker({ addWaypoint, mode }) {
    useMapEvents({
        click(e) {
            // Only add points directly if NOT in 'area' mode (or handled differently)
            // But here we reuse the same clicker for both, just storing in different lists state upstairs
            addWaypoint(e.latlng);
        },
    });
    return null;
}

function DroneTracker({ position, followMode }) {
    const map = useMap();
    const [hasInitialized, setHasInitialized] = React.useState(false);

    useEffect(() => {
        // Auto-center on first valid GPS lock
        if (!hasInitialized && position.lat !== 0 && position.lat !== 16.506) {
            map.setView([position.lat, position.lon], 18, { animate: true });
            setHasInitialized(true);
        }
        // Regular follow mode
        else if (followMode && position.lat !== 0 && position.lat !== 16.506) {
            map.setView([position.lat, position.lon], map.getZoom(), { animate: true });
        }
    }, [position, followMode, map, hasInitialized]);
    return null;
}

function App() {
    const [waypoints, setWaypoints] = useState([]); // Mission Points
    const [polygonPoints, setPolygonPoints] = useState([]); // Area Points
    const [drawMode, setDrawMode] = useState('mission'); // 'mission' or 'area'

    const [scout, setScout] = useState({
        lat: 16.506, lon: 80.648, alt: 0, bat: 0, bat_voltage: 0, bat_current: 0, bat_time_min: 0, gps_sats: 0,
        status: "DISARMED", speed: 0, mode: "UNKNOWN"
    });
    const [delivery, setDelivery] = useState({
        lat: 16.506, lon: 80.648, alt: 0, bat: 0, bat_voltage: 0, bat_current: 0, bat_time_min: 0, gps_sats: 0,
        status: "DISARMED", speed: 0, mode: "UNKNOWN"
    });
    const [survivor, setSurvivor] = useState(null);
    const [detections, setDetections] = useState([]);

    const [takeoffAlt, setTakeoffAlt] = useState(10);
    const [activeDrone, setActiveDrone] = useState('scout');
    const [followMode, setFollowMode] = useState(true);
    const [showCameraFeed, setShowCameraFeed] = useState(true);
    const [viewMode, setViewMode] = useState('map'); // 'map' or 'camera'

    // Mission Settings
    const [missionAlt, setMissionAlt] = useState(20);
    const [missionSpeed, setMissionSpeed] = useState(5);

    // Auto-update defaults when switching drones
    useEffect(() => {
        if (activeDrone === 'delivery') {
            setMissionAlt(6.7); // ~22ft
            setMissionSpeed(2.5);
        } else {
            // Scout defaults
            setMissionAlt(20);
            setMissionSpeed(5);
        }
    }, [activeDrone]);

    // RC Override State [Roll, Pitch, Thr, Yaw, 0, 0, 0, 0]
    const [rcChannels, setRcChannels] = useState([1500, 1500, 1000, 1500, 0, 0, 0, 0]);

    const updateRc = (idx, val) => {
        const newCh = [...rcChannels];
        newCh[idx] = parseInt(val);
        setRcChannels(newCh);
    };

    const sendRcOverride = () => {
        sendCommand('rc_override', { channels: rcChannels });
    };

    const releaseRc = () => {
        setRcChannels([1500, 1500, 1000, 1500, 0, 0, 0, 0]); // Reset UI
        sendCommand('rc_override', { channels: [0, 0, 0, 0, 0, 0, 0, 0] }); // Send 0 to release to FC
    };

    useEffect(() => {
        socket.on('telemetry_update', (msg) => {
            if (msg.drone === 'scout') setScout({ ...msg.data, lastHeartbeat: Date.now() });
            if (msg.drone === 'delivery') setDelivery({ ...msg.data, lastHeartbeat: Date.now() });

            // Listen for survivor data inside telemetry or separate event
            if (msg.drone === 'scout' && msg.data.survivor_detected) {
                setSurvivor([msg.data.lat, msg.data.lon]);
            }
        });

        socket.on('connect', () => console.log("✅ Socket.IO Connected!"));
        socket.on('disconnect', () => console.log("❌ Socket.IO Disconnected!"));

        socket.on('survivor_alert', (msg) => {
            console.log("🚨 SURVIVOR FOUND:", msg);
            setSurvivor([msg.lat, msg.lon]);
        });

        // Listen for human detections
        socket.on('detection_update', (detection) => {
            console.log("👤 HUMAN DETECTED:", detection);
            setDetections(prev => [...prev, detection]);
        });

        socket.on('auto_dispatch_alert', (msg) => {
            alert(`🚀 AUTO-DISPATCH INITIATED!\nDelivery Drone sending to: ${msg.lat.toFixed(5)}, ${msg.lon.toFixed(5)}`);
            setActiveDrone('delivery'); // Switch to delivery drone view
        });

        // Fetch existing detections on mount
        axios.get(`http://${IP_ADDRESS}:5000/detections`)
            .then(res => setDetections(res.data.detections || []))
            .catch(err => console.error("Failed to fetch detections:", err));

        return () => {
            socket.off('telemetry_update');
            socket.off('survivor_alert');
            socket.off('detection_update');
            socket.off('auto_dispatch_alert');
        };
    }, []);

    const sendCommand = (cmd, payload = {}) => {
        axios.post(`http://${IP_ADDRESS}:5000/${cmd}`, payload)
            .then(() => alert(`✅ Command ${cmd} Sent`))
            .catch(() => alert("❌ Backend Error"));
    };


    const uploadMission = () => {
        if (waypoints.length === 0) return alert("Select points first!");
        // Send waypoints + mission settings
        axios.post(`http://${IP_ADDRESS}:5000/upload_mission`, {
            waypoints,
            alt: parseFloat(missionAlt),
            speed: parseFloat(missionSpeed),
            drone: activeDrone
        })
            .then(() => alert("✅ Mission Uploaded!"))
            .catch(() => alert("❌ Backend Error"));
    };

    const launchMission = () => {
        // Switch to AUTO mode to start mission
        sendCommand('set_mode', { mode: 'AUTO' });
    };

    const handleKMLUpload = (event) => {
        const file = event.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = (e) => {
            const text = e.target.result;
            try {
                const parser = new DOMParser();
                const xmlDoc = parser.parseFromString(text, "text/xml");
                const coordinates = xmlDoc.getElementsByTagName("coordinates");
                const newPoints = [];

                for (let i = 0; i < coordinates.length; i++) {
                    const coordText = coordinates[i].textContent.trim();
                    const coordPairs = coordText.split(/\s+/);

                    coordPairs.forEach(pair => {
                        const parts = pair.split(',');
                        if (parts.length >= 2) {
                            // KML is Longitude, Latitude
                            const lng = parseFloat(parts[0]);
                            const lat = parseFloat(parts[1]);
                            if (!isNaN(lat) && !isNaN(lng)) {
                                newPoints.push({ lat, lng });
                            }
                        }
                    });
                }

                if (newPoints.length > 0) {
                    setWaypoints(newPoints);
                    alert(`✅ Loaded ${newPoints.length} points from KML`);
                } else {
                    alert("❌ No valid coordinates found in KML");
                }
            } catch (err) {
                console.error(err);
                alert("❌ Error parsing KML file");
            }
        };
        reader.readAsText(file);
    };

    const handleMapClick = (latlng) => {
        if (drawMode === 'mission') {
            setWaypoints([...waypoints, latlng]);
        } else if (drawMode === 'area') {
            setPolygonPoints([...polygonPoints, latlng]);
        }
    };

    const generatePath = () => {
        if (polygonPoints.length < 3) return alert("Draw a polygon first (3+ points)!");
        const grid = generateGrid(polygonPoints);
        setWaypoints(grid);
        setDrawMode('mission'); // Switch back to visualize points
        alert(`✅ Generated ${grid.length} Search Points!`);
    };

    const ModeBadge = ({ mode }) => (
        <span className="px-2 py-0.5 rounded text-[10px] font-bold bg-blue-100 text-blue-700">
            {mode}
        </span>
    );

    const Badge = ({ status }) => (
        <span className={`px-2 py-0.5 rounded text-[10px] font-bold ${status === 'ARMED' ? 'bg-green-100 text-green-700' : 'bg-slate-100 text-slate-500'}`}>
            {status}
        </span>
    );

    const activeData = activeDrone === 'scout' ? scout : delivery;

    const handleRescue = () => {
        if (!survivor) return;
        sendCommand('rescue', { lat: survivor[0], lon: survivor[1] });
        setSurvivor(null); // Clear after launch
        setActiveDrone('delivery'); // Switch view to delivery drone
    };

    const dispatchDelivery = (lat, lon) => {
        axios.post(`http://${IP_ADDRESS}:5000/dispatch_delivery`, { lat, lon })
            .then(() => {
                alert(`✅ Delivery Dispatched to (${lat.toFixed(5)}, ${lon.toFixed(5)})`);
                setActiveDrone('delivery'); // Switch to delivery view
            })
            .catch(() => alert("❌ Dispatch Failed"));
    };

    const clearDetections = () => {
        axios.post(`http://${IP_ADDRESS}:5000/clear_detections`)
            .then(() => {
                setDetections([]);
                alert("✅ Detections Cleared");
            })
            .catch(() => alert("❌ Clear Failed"));
    };

    return (
        <div className="flex h-screen w-screen overflow-hidden bg-slate-50">

            {/* SURVIVOR ALERT MODAL */}
            {survivor && (
                <div className="absolute inset-0 z-[2000] bg-black/50 flex items-center justify-center backdrop-blur-sm">
                    <div className="bg-white p-6 rounded-xl shadow-2xl max-w-sm w-full border-l-4 border-orange-500 animate-bounce-in">
                        <div className="flex items-center gap-3 mb-4">
                            <div className="bg-orange-100 p-2 rounded-full">
                                <span className="text-2xl">🚨</span>
                            </div>
                            <div>
                                <h2 className="text-lg font-bold text-slate-900">Survivor Detected!</h2>
                                <p className="text-xs text-slate-500">Location: {survivor[0].toFixed(5)}, {survivor[1].toFixed(5)}</p>
                            </div>
                        </div>

                        <div className="bg-slate-50 p-3 rounded mb-4 text-xs text-slate-600">
                            <strong>Mission Profile:</strong> Autonomous delivery. Drone will descend to 5m, drop kit, and RTL.
                        </div>

                        <div className="flex gap-3">
                            <button
                                onClick={() => setSurvivor(null)}
                                className="flex-1 py-2 px-4 rounded-lg border border-slate-200 text-slate-600 font-semibold hover:bg-slate-50 text-sm"
                            >
                                Dismiss
                            </button>
                            <button
                                onClick={handleRescue}
                                className="flex-1 py-2 px-4 rounded-lg bg-orange-500 text-white font-bold hover:bg-orange-600 shadow-lg shadow-orange-500/30 text-sm animate-pulse"
                            >
                                🚀 LAUNCH RESCUE
                            </button>
                        </div>
                    </div>
                </div>
            )}

            <aside className="w-80 flex-shrink-0 border-r border-slate-200 bg-white flex flex-col z-10">
                <div className="p-4 border-b border-slate-100 flex justify-between items-center">
                    <h1 className="text-lg font-bold text-slate-900 tracking-tight">SkyLink<span className="text-blue-500">.NIDAR</span></h1>
                    <div className="flex gap-2">
                        <button
                            onClick={() => setViewMode('map')}
                            className={`px-3 py-1 rounded text-xs font-semibold transition ${viewMode === 'map' ? 'bg-blue-500 text-white' : 'bg-slate-100 text-slate-600 hover:bg-slate-200'}`}
                        >
                            🗺️ Map
                        </button>
                        <button
                            onClick={() => setViewMode('camera')}
                            className={`px-3 py-1 rounded text-xs font-semibold transition ${viewMode === 'camera' ? 'bg-blue-500 text-white' : 'bg-slate-100 text-slate-600 hover:bg-slate-200'}`}
                        >
                            📹 Camera
                        </button>
                    </div>
                </div>

                <div className="flex-1 overflow-y-auto p-4 space-y-4">

                    <Card title="Fleet Status">
                        {/* Scout Card */}
                        <div
                            className={`p-3 rounded-md cursor-pointer border transition-colors mb-2 ${activeDrone === 'scout' ? 'bg-blue-50 border-blue-200' : 'bg-slate-50 border-transparent hover:bg-slate-100'}`}
                            onClick={() => setActiveDrone('scout')}
                        >
                            <div className="flex justify-between items-center mb-2">
                                <div className="flex items-center gap-2">
                                    <div className={`w-2 h-2 rounded-full ${Date.now() - (scout.lastHeartbeat || 0) < 3000 ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></div>
                                    <span className="font-semibold text-sm">Scout One</span>
                                </div>
                                <div className="flex gap-1">
                                    <ModeBadge mode={scout.mode} />
                                    <Badge status={scout.status} />
                                </div>
                            </div>
                            <div className="text-xs text-slate-500 space-y-1">
                                <div className="grid grid-cols-2 gap-x-3 gap-y-1">
                                    <div>Alt: <span className="text-slate-900 font-medium">{scout.alt.toFixed(1)}m</span></div>
                                    <div>Speed: <span className="text-slate-900 font-medium">{scout.speed}m/s</span></div>
                                    <div>Battery: <span className="text-slate-900 font-medium">{scout.bat}%</span></div>
                                    <div>Voltage: <span className="text-slate-900 font-medium">{scout.bat_voltage ? scout.bat_voltage.toFixed(1) + 'V' : '---'}</span></div>
                                    <div className="col-span-2">🛰️ GPS: <span className={`font-semibold ${scout.gps_sats >= 10 ? 'text-green-600' : scout.gps_sats >= 6 ? 'text-yellow-600' : 'text-red-600'}`}>{scout.gps_sats} sats</span></div>
                                </div>
                                {scout.bat_time_min > 0 && (
                                    <div className="pt-1 border-t border-slate-200">
                                        ⏱️ Est. Time: <span className="text-blue-600 font-semibold">{scout.bat_time_min.toFixed(1)}min</span>
                                    </div>
                                )}
                            </div>
                        </div>

                        {/* Delivery Card */}
                        <div
                            className={`p-3 rounded-md cursor-pointer border transition-colors ${activeDrone === 'delivery' ? 'bg-blue-50 border-blue-200' : 'bg-slate-50 border-transparent hover:bg-slate-100'}`}
                            onClick={() => setActiveDrone('delivery')}
                        >
                            <div className="flex justify-between items-center mb-2">
                                <div className="flex items-center gap-2">
                                    <div className={`w-2 h-2 rounded-full ${Date.now() - (delivery.lastHeartbeat || 0) < 3000 ? 'bg-green-500 animate-pulse' : 'bg-red-500'}`}></div>
                                    <span className="font-semibold text-sm">Delivery Two</span>
                                </div>
                                <div className="flex gap-1">
                                    <ModeBadge mode={delivery.mode} />
                                    <Badge status={delivery.status} />
                                </div>
                            </div>
                            <div className="text-xs text-slate-500 space-y-1">
                                <div className="grid grid-cols-2 gap-x-3 gap-y-1">
                                    <div>Alt: <span className="text-slate-900 font-medium">{delivery.alt.toFixed(1)}m</span></div>
                                    <div>Speed: <span className="text-slate-900 font-medium">{delivery.speed}m/s</span></div>
                                    <div>Battery: <span className="text-slate-900 font-medium">{delivery.bat}%</span></div>
                                    <div>Voltage: <span className="text-slate-900 font-medium">{delivery.bat_voltage ? delivery.bat_voltage.toFixed(1) + 'V' : '---'}</span></div>
                                    <div className="col-span-2">🛰️ GPS: <span className={`font-semibold ${delivery.gps_sats >= 10 ? 'text-green-600' : delivery.gps_sats >= 6 ? 'text-yellow-600' : 'text-red-600'}`}>{delivery.gps_sats} sats</span></div>
                                </div>
                                {delivery.bat_time_min > 0 && (
                                    <div className="pt-1 border-t border-slate-200">
                                        ⏱️ Est. Time: <span className="text-blue-600 font-semibold">{delivery.bat_time_min.toFixed(1)}min</span>
                                    </div>
                                )}
                            </div>
                        </div>
                    </Card>

                    <Card title={`Control: ${activeDrone.toUpperCase()}`}>
                        <div className="grid grid-cols-3 gap-2 mb-4">
                            <Button variant="success" onClick={() => sendCommand('arm')}>ARM</Button>
                            <Button variant="warning" onClick={() => sendCommand('disarm')}>DISARM</Button>
                            <Button variant="danger" onClick={() => sendCommand('land')}>LAND</Button>
                        </div>
                        <div className="mb-4">
                            <Button variant="secondary" className="w-full text-xs" onClick={() => sendCommand('indoor_mode')}>🛡️ SET INDOOR MODE</Button>
                        </div>

                        {/* MODE SELECTOR */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2 flex gap-2">
                            <select
                                id="mode-select"
                                className="w-full text-xs h-8 bg-white border border-slate-200 rounded px-2 outline-none"
                            >
                                <option value="STABILIZE">STABILIZE</option>
                                <option value="LOITER">LOITER</option>
                                <option value="GUIDED">GUIDED</option>
                                <option value="RTL">RTL</option>
                                <option value="AUTO">AUTO</option>
                                <option value="LAND">LAND</option>
                            </select>
                            <Button
                                variant="secondary"
                                className="py-1 px-3 text-xs"
                                onClick={() => sendCommand('set_mode', { mode: document.getElementById('mode-select').value })}
                            >
                                SET
                            </Button>
                        </div>





                        {/* RC OVERRIDE CONTROLS */}
                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2">
                            <h4 className="text-[10px] font-bold text-slate-400 uppercase mb-2">Manual Override (RC)</h4>

                            <div className="space-y-2 mb-2">
                                <RcSlider label="Roll (1)" val={rcChannels[0]} setVal={(v) => updateRc(0, v)} />
                                <RcSlider label="Pitch (2)" val={rcChannels[1]} setVal={(v) => updateRc(1, v)} />
                                <RcSlider label="Thr (3)" val={rcChannels[2]} setVal={(v) => updateRc(2, v)} />
                                <RcSlider label="Yaw (4)" val={rcChannels[3]} setVal={(v) => updateRc(3, v)} />
                            </div>

                            <div className="flex gap-2">
                                <Button variant="secondary" className="flex-1 text-[10px]" onClick={sendRcOverride}>TEST SEND</Button>
                                <Button variant="danger" className="flex-1 text-[10px]" onClick={releaseRc}>RELEASE</Button>
                            </div>
                        </div>

                        <div className="bg-slate-50 p-2 rounded border border-slate-200 mb-2">
                            <label className="text-[10px] uppercase font-bold text-slate-400 block mb-1">Takeoff Altitude (m)</label>
                            <div className="flex gap-2">
                                <input
                                    type="number"
                                    value={takeoffAlt}
                                    onChange={(e) => setTakeoffAlt(e.target.value)}
                                    className="w-full text-sm bg-white border border-slate-200 rounded px-2 outline-none focus:border-blue-500"
                                />
                                <Button variant="secondary" className="py-1 px-3 text-xs" onClick={() => sendCommand('takeoff', { alt: takeoffAlt })}>GO</Button>
                            </div>
                        </div>
                        <div className="flex items-center justify-between p-2 bg-slate-50 rounded border border-slate-200">
                            <span className="text-xs font-semibold text-slate-700">Auto-Follow Map</span>
                            <button
                                onClick={() => setFollowMode(!followMode)}
                                className={`w-10 h-5 rounded-full relative transition-colors ${followMode ? 'bg-blue-500' : 'bg-slate-300'}`}
                            >
                                <div className={`absolute top-1 w-3 h-3 bg-white rounded-full transition-all ${followMode ? 'left-6' : 'left-1'}`} />
                            </button>
                        </div>
                    </Card>

                    {/* SMART PLANNER */}
                    <Card title="Start Planner">
                        <div className="flex gap-2 mb-2">
                            <Button
                                variant={drawMode === 'mission' ? 'primary' : 'secondary'}
                                onClick={() => setDrawMode('mission')}
                                className="flex-1 text-xs"
                            >
                                Points
                            </Button>
                            <Button
                                variant={drawMode === 'area' ? 'warning' : 'secondary'}
                                onClick={() => setDrawMode('area')}
                                className="flex-1 text-xs"
                            >
                                Area (Box)
                            </Button>
                        </div>

                        {drawMode === 'area' ? (
                            <div className="space-y-2">
                                <div className="text-xs text-slate-500">
                                    Click map to draw search area ({polygonPoints.length} pts).
                                </div>
                                <Button variant="primary" onClick={generatePath} className="w-full">
                                    Generate Grid
                                </Button>
                                <Button variant="secondary" onClick={() => setPolygonPoints([])} className="w-full">
                                    Clear Area
                                </Button>
                            </div>
                        ) : (
                            <div className="space-y-2">
                                <div className="text-xs text-slate-500">
                                    {waypoints.length} waypoints ready.
                                </div>

                                {/* Mission Parameter Inputs */}
                                <div className="grid grid-cols-2 gap-2 mb-2">
                                    <div>
                                        <label className="text-[10px] font-bold text-slate-500 block mb-1">
                                            Altitude (m) <span className="font-normal text-slate-400">≈ {(parseFloat(missionAlt || 0) * 3.28).toFixed(1)}ft</span>
                                        </label>
                                        <input
                                            type="number"
                                            value={missionAlt}
                                            onChange={(e) => setMissionAlt(e.target.value)}
                                            className="w-full text-xs border border-slate-200 rounded px-2 py-1"
                                        />
                                    </div>
                                    <div>
                                        <label className="text-[10px] font-bold text-slate-500 block mb-1">Speed (m/s)</label>
                                        <input
                                            type="number"
                                            value={missionSpeed}
                                            onChange={(e) => setMissionSpeed(e.target.value)}
                                            className="w-full text-xs border border-slate-200 rounded px-2 py-1"
                                        />
                                    </div>
                                </div>

                                <Button variant="success" onClick={uploadMission} className="w-full text-xs py-2">
                                    📤 Upload Mission
                                </Button>

                                <Button variant="primary" onClick={launchMission} className="w-full text-xs py-2 font-bold animate-pulse">
                                    🚀 Launch Mission
                                </Button>

                                <Button variant="secondary" onClick={() => setWaypoints([])} className="w-full text-xs">
                                    Clear Points
                                </Button>
                                <div className="mt-2 text-xs text-slate-400 text-center">- OR -</div>
                                <label className="block w-full text-center p-2 border border-dashed border-slate-300 rounded cursor-pointer hover:bg-slate-100 mt-2">
                                    <span className="text-xs text-slate-600">📂 Import KML File</span>
                                    <input type="file" accept=".kml" className="hidden" onChange={handleKMLUpload} />
                                </label>
                            </div>
                        )}
                    </Card>

                    {/* HUMAN DETECTIONS */}
                    < Card title="🚨 Human Detections" >
                        <div className="space-y-2">
                            {detections.length === 0 ? (
                                <div className="text-xs text-slate-400 text-center py-4">
                                    No detections yet. Scout drone will auto-detect humans.
                                </div>
                            ) : (
                                <>
                                    <div className="max-h-60 overflow-y-auto space-y-2">
                                        {detections.map((det, idx) => (
                                            <div key={idx} className="bg-green-50 border border-green-200 rounded-lg p-3">
                                                <div className="flex justify-between items-start mb-2">
                                                    <div className="flex items-center gap-2">
                                                        <span className="text-lg">👤</span>
                                                        <div>
                                                            <div className="text-xs font-semibold text-green-800">
                                                                Detection #{idx + 1}
                                                            </div>
                                                            <div className="text-[10px] text-slate-500">
                                                                {new Date(det.timestamp).toLocaleTimeString()}
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                                <div className="text-xs font-mono text-slate-600 mb-2">
                                                    📍 {det.lat.toFixed(6)}, {det.lon.toFixed(6)}
                                                </div>
                                                <Button
                                                    variant="success"
                                                    className="w-full text-xs py-1"
                                                    onClick={() => dispatchDelivery(det.lat, det.lon)}
                                                >
                                                    🚁 Dispatch Delivery
                                                </Button>
                                            </div>
                                        ))}
                                    </div>
                                    <Button
                                        variant="secondary"
                                        className="w-full text-xs"
                                        onClick={clearDetections}
                                    >
                                        Clear All ({detections.length})
                                    </Button>
                                </>
                            )}
                        </div>
                    </Card >

                    {/* CAMERA FEED */}
                    {
                        showCameraFeed && (
                            <Card title="📹 Scout Camera Feed">
                                <div className="space-y-2">
                                    <div className="relative bg-black rounded overflow-hidden" style={{ aspectRatio: '16/9' }}>
                                        <img
                                            src={`http://${IP_ADDRESS}:5001/video_feed`}
                                            alt="Scout Camera"
                                            className="w-full h-full object-contain"
                                            onError={(e) => {
                                                e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='640' height='360'%3E%3Crect fill='%23111' width='640' height='360'/%3E%3Ctext x='50%25' y='50%25' fill='%23666' font-size='20' text-anchor='middle' dominant-baseline='middle'%3ECamera Offline%3C/text%3E%3C/svg%3E";
                                            }}
                                        />
                                        <div className="absolute top-2 right-2 bg-red-500 text-white px-2 py-1 rounded-full text-xs font-bold flex items-center gap-1 animate-pulse">
                                            <div className="w-2 h-2 bg-white rounded-full"></div>
                                            LIVE
                                        </div>
                                        <div className="absolute bottom-2 left-2 bg-black/70 text-white px-2 py-1 rounded text-xs">
                                            720p • 24fps • HOG Detection
                                        </div>
                                    </div>
                                    <Button
                                        variant="secondary"
                                        className="w-full text-xs"
                                        onClick={() => setShowCameraFeed(false)}
                                    >
                                        Hide Camera
                                    </Button>
                                </div>
                            </Card>
                        )
                    }

                    {
                        !showCameraFeed && (
                            <Button
                                variant="primary"
                                className="w-full"
                                onClick={() => setShowCameraFeed(true)}
                            >
                                📹 Show Camera Feed
                            </Button>
                        )
                    }

                </div >
            </aside >

            {/* MAIN CONTENT - Map or Camera View */}
            {
                viewMode === 'map' ? (
                    <main className="flex-1 relative">
                        <MapContainer center={[16.5062, 80.6480]} zoom={16} zoomControl={false} style={{ height: "100%", width: "100%" }}>
                            <TileLayer url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png" />
                            <MapClicker addWaypoint={handleMapClick} mode={drawMode} />

                            <DroneTracker position={activeData} followMode={followMode} />

                            {/* Waypoints Path */}
                            {waypoints.map((wp, i) => <Marker key={i} position={wp} />)}
                            <Polyline positions={waypoints} color="#3b82f6" weight={3} />

                            {/* Area Polygon */}
                            {polygonPoints.length > 0 && (
                                <>
                                    {polygonPoints.map((p, i) => <Marker key={`poly-${i}`} position={p} icon={scoutIcon} opacity={0.5} />)}
                                    <Polygon positions={polygonPoints} color="orange" dashArray="5, 5" />
                                </>
                            )}

                            <Marker position={[scout.lat, scout.lon]} icon={scoutIcon}><Popup>Scout</Popup></Marker>
                            <Marker position={[delivery.lat, delivery.lon]} icon={deliveryIcon}><Popup>Delivery</Popup></Marker>
                            {survivor && <Marker position={survivor} icon={survivorIcon}><Popup>SURVIVOR FOUND!</Popup></Marker>}

                            {/* Human Detection Markers */}
                            {detections.map((det, idx) => (
                                <Marker key={`detection-${idx}`} position={[det.lat, det.lon]} icon={detectionIcon}>
                                    <Popup>
                                        <div className="text-center">
                                            <div className="font-bold text-sm mb-1">👤 Human Detected</div>
                                            <div className="text-xs text-slate-600 mb-2">
                                                {new Date(det.timestamp).toLocaleString()}
                                            </div>
                                            <button
                                                className="bg-green-500 text-white px-3 py-1 rounded text-xs font-semibold hover:bg-green-600"
                                                onClick={() => dispatchDelivery(det.lat, det.lon)}
                                            >
                                                Dispatch Delivery
                                            </button>
                                        </div>
                                    </Popup>
                                </Marker>
                            ))}

                        </MapContainer>

                        <div className="absolute top-4 right-4 bg-white/90 backdrop-blur px-3 py-1 rounded shadow text-xs font-mono z-[1000] flex gap-2">
                            <span>System Ready • Online</span>
                            {drawMode === 'area' && <span className="text-orange-500 font-bold">• Drawing Area</span>}
                        </div>

                        {/* GPS Coordinates Display */}
                        <div className="absolute bottom-4 left-4 bg-white/95 backdrop-blur px-3 py-2 rounded-lg shadow-lg z-[1000] border border-slate-200">
                            <div className="text-[10px] font-bold text-slate-400 uppercase mb-1">Live GPS Position</div>
                            <div className="space-y-0.5">
                                <div className="text-xs font-mono">
                                    <span className="text-blue-600 font-semibold">Scout:</span>
                                    <span className="ml-2 text-slate-700">{scout.lat === 0 ? 'No GPS' : `${scout.lat.toFixed(6)}°N, ${scout.lon.toFixed(6)}°E`}</span>
                                </div>
                                <div className="text-xs font-mono">
                                    <span className="text-red-600 font-semibold">Deliv:</span>
                                    <span className="ml-2 text-slate-700">{delivery.lat === 0 ? 'No GPS' : `${delivery.lat.toFixed(6)}°N, ${delivery.lon.toFixed(6)}°E`}</span>
                                </div>
                            </div>
                        </div>
                    </main>
                ) : (
                    /* CAMERA VIEW PAGE */
                    <main className="flex-1 bg-slate-900 flex flex-col">
                        <div className="flex-1 flex items-center justify-center p-8">
                            <div className="w-full max-w-6xl">
                                <div className="relative bg-black rounded-lg overflow-hidden shadow-2xl">
                                    <img
                                        src={`http://${scout.ip || IP_ADDRESS}:5001/video_feed`}
                                        alt="Scout Camera Feed"
                                        className="w-full"
                                        style={{ maxHeight: '70vh', objectFit: 'contain' }}
                                        onError={(e) => {
                                            e.target.src = "data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='1280' height='720'%3E%3Crect fill='%23111' width='1280' height='720'/%3E%3Ctext x='50%25' y='50%25' fill='%23666' font-size='24' text-anchor='middle' dominant-baseline='middle'%3ECamera Offline%3C/text%3E%3C/svg%3E";
                                        }}
                                    />
                                    <div className="absolute top-4 right-4 bg-red-500 text-white px-3 py-2 rounded-full text-sm font-bold flex items-center gap-2 animate-pulse shadow-lg">
                                        <div className="w-3 h-3 bg-white rounded-full"></div>
                                        LIVE ({scout.ip || 'Connecting...'})
                                    </div>
                                    <div className="absolute bottom-4 left-4 bg-black/80 text-white px-3 py-2 rounded text-sm font-mono">
                                        📹 720p HD • 24fps • HOG Detection Active
                                    </div>
                                    <div className="absolute top-4 left-4 bg-black/80 text-white px-3 py-2 rounded space-y-1">
                                        <div className="text-xs font-bold text-green-400">Scout Drone</div>
                                        <div className="text-xs">🛰️ GPS: {scout.gps_sats} sats</div>
                                        <div className="text-xs">📍 Detections: {detections.length}</div>
                                    </div>
                                </div>
                            </div>
                        </div>

                        {/* Detection Locations List */}
                        <div className="bg-slate-800 border-t border-slate-700 p-6">
                            <div className="max-w-6xl mx-auto">
                                <h2 className="text-white text-lg font-bold mb-4 flex items-center gap-2">
                                    <span>🎯</span>
                                    Detected Human Locations ({detections.length})
                                </h2>
                                {detections.length === 0 ? (
                                    <div className="text-slate-400 text-center py-8">
                                        No humans detected yet. Detection system is active.
                                    </div>
                                ) : (
                                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                                        {detections.map((det, idx) => (
                                            <div key={idx} className="bg-slate-700 rounded-lg p-4 border border-green-500/50">
                                                <div className="flex items-start justify-between mb-3">
                                                    <div className="flex items-center gap-2">
                                                        <span className="text-2xl">👤</span>
                                                        <div>
                                                            <div className="text-white font-bold">Detection #{detections.length - idx}</div>
                                                            <div className="text-xs text-slate-400">
                                                                {new Date(det.timestamp).toLocaleTimeString()}
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                                <div className="bg-slate-800 rounded p-2 mb-3">
                                                    <div className="text-xs text-slate-400 mb-1">GPS Coordinates</div>
                                                    <div className="font-mono text-sm text-green-400">
                                                        {det.lat.toFixed(6)}°N<br />
                                                        {det.lon.toFixed(6)}°E
                                                    </div>
                                                </div>
                                                <button
                                                    onClick={() => dispatchDelivery(det.lat, det.lon)}
                                                    className="w-full bg-green-600 hover:bg-green-700 text-white py-2 px-4 rounded font-semibold text-sm transition"
                                                >
                                                    🚁 Dispatch Delivery
                                                </button>
                                            </div>
                                        ))}
                                    </div>
                                )}
                            </div>
                        </div>
                    </main>
                )
            }



        </div >
    );
}

export default App;
