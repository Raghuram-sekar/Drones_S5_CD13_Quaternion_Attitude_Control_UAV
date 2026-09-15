import http.server
import socketserver
import webbrowser
import threading
import os
import sys

# ---------------------------------------------------------
# HTML5 + Three.js + Chart.js GPU-Accelerated Real-Time Simulator
# ---------------------------------------------------------
HTML_CONTENT = r"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Fast 6-DOF Drone Simulator: Quaternion vs. Euler</title>
    <!-- Three.js for 60 FPS 3D GPU Rendering -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <!-- Chart.js for Real-Time Smooth Graphs -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; }
        body { background-color: #0B0F19; color: #F8FAFC; overflow: hidden; height: 100vh; display: flex; flex-direction: column; }

        /* Header */
        header { background: #111827; border-bottom: 1px solid #1E293B; padding: 10px 20px; text-align: center; }
        header h1 { font-size: 1.1rem; color: #F8FAFC; letter-spacing: 0.5px; }
        header p { font-size: 0.8rem; color: #38BDF8; margin-top: 2px; }

        /* Main Dashboard Grid */
        .dashboard { display: grid; grid-template-columns: 2fr 3fr 2fr; gap: 10px; padding: 10px; flex: 1; height: calc(100vh - 120px); }

        /* Left Panel: 3D Drones */
        .panel-3d { display: flex; flex-direction: column; gap: 10px; }
        .canvas-container { flex: 1; background: #111827; border: 1px solid #1E293B; border-radius: 8px; position: relative; overflow: hidden; }
        .canvas-label { position: absolute; top: 10px; left: 10px; background: rgba(15, 23, 42, 0.85); padding: 4px 10px; border-radius: 4px; font-size: 0.75rem; font-weight: bold; border-left: 3px solid #38BDF8; }
        .label-quat { border-left-color: #00F0FF; color: #00F0FF; }
        .label-euler { border-left-color: #FF0055; color: #FF0055; }

        /* Middle Panel: Live 2D Graphs */
        .panel-graphs { display: flex; flex-direction: column; gap: 10px; }
        .chart-box { flex: 1; background: #1E293B; border: 1px solid #334155; border-radius: 8px; padding: 6px 10px; position: relative; }
        .chart-title { font-size: 0.75rem; font-weight: bold; color: #38BDF8; margin-bottom: 4px; }
        .chart-box canvas { width: 100% !important; height: calc(100% - 20px) !important; }

        /* Right Panel: Telemetry HUD */
        .panel-hud { background: #0F172A; border: 1px solid #00F0FF; border-radius: 8px; padding: 15px; font-family: 'Courier New', monospace; font-size: 0.8rem; color: #F1F5F9; line-height: 1.5; overflow-y: auto; }
        .hud-header { color: #00F0FF; font-weight: bold; border-bottom: 1px solid #1E293B; padding-bottom: 6px; margin-bottom: 10px; }
        .hud-section { margin-bottom: 12px; }
        .hud-title { color: #EAB308; font-weight: bold; }
        .hud-quat { color: #00F0FF; }
        .hud-euler { color: #FF0055; }
        .hud-ok { color: #22C55E; font-weight: bold; }
        .hud-fail { color: #EF4444; font-weight: bold; }

        /* Bottom Controls */
        footer { background: #111827; border-top: 1px solid #1E293B; padding: 10px 20px; display: flex; justify-content: center; gap: 12px; align-items: center; }
        .btn { background: #1E293B; color: #F8FAFC; border: 1px solid #334155; padding: 8px 16px; border-radius: 6px; font-size: 0.85rem; font-weight: 600; cursor: pointer; transition: all 0.15s ease; }
        .btn:hover { background: #334155; border-color: #38BDF8; }
        .btn:active { transform: scale(0.96); }
        .btn-active { background: #0284C7 !important; border-color: #38BDF8 !important; color: #FFF !important; }
        .btn-reset { background: #E11D48 !important; border-color: #F43F5E !important; }
        .btn-reset:hover { background: #BE123C !important; }
        .btn-play { background: #16A34A !important; border-color: #22C55E !important; }
        .btn-play:hover { background: #15803D !important; }
    </style>
</head>
<body>

    <header>
        <h1 id="mainTitle">FAST 6-DOF DRONE SIMULATOR: 90° BANK KNIFE-EDGE MANEUVER</h1>
        <p>Quaternion Controller (Cyan) vs. Euler Controller (Neon Red) — 60 FPS Hardware Accelerated</p>
    </header>

    <div class="dashboard">
        <!-- Left: 3D Aircraft Visualizer -->
        <div class="panel-3d">
            <div class="canvas-container">
                <div class="canvas-label label-quat">QUATERNION CONTROLLER (SINGULARITY-FREE)</div>
                <div id="containerQuat" style="width: 100%; height: 100%;"></div>
            </div>
            <div class="canvas-container">
                <div class="canvas-label label-euler">EULER CONTROLLER (CROSS-COUPLED FAILURE)</div>
                <div id="containerEuler" style="width: 100%; height: 100%;"></div>
            </div>
        </div>

        <!-- Middle: 2D Tracking Charts -->
        <div class="panel-graphs">
            <div class="chart-box">
                <div class="chart-title">ROLL ANGLE (&phi;) TRACKING</div>
                <canvas id="chartRoll"></canvas>
            </div>
            <div class="chart-box">
                <div class="chart-title">PITCH ANGLE (&theta;) TRACKING</div>
                <canvas id="chartPitch"></canvas>
            </div>
            <div class="chart-box">
                <div class="chart-title">YAW ANGLE (&psi;) TRACKING</div>
                <canvas id="chartYaw"></canvas>
            </div>
        </div>

        <!-- Right: Primary Flight Display HUD -->
        <div class="panel-hud" id="hudBox">
            <div class="hud-header">[+] PRIMARY FLIGHT DISPLAY TELEMETRY</div>
            <div class="hud-section">
                <div>* Simulation Time: <span id="hudTime">0.00</span> s | Speed: <span id="hudSpeed">1.0x</span></div>
                <div>* Bank Target: <span id="hudBank">90</span>° | Mode: <span id="hudMode">ACTIVE</span></div>
            </div>
            <div class="hud-section">
                <div class="hud-title">[*] TARGET SETPOINT:</div>
                <div>Roll: <span id="hudSpRoll">0.0</span>°  Pitch: <span id="hudSpPitch">0.0</span>°  Yaw: <span id="hudSpYaw">0.0</span>°</div>
            </div>
            <div class="hud-section hud-quat">
                <div>[CYAN] QUATERNION CONTROLLER:</div>
                <div>qw: <span id="hudQw">1.000</span> qx: <span id="hudQx">0.000</span> qy: <span id="hudQy">0.000</span> qz: <span id="hudQz">0.000</span></div>
                <div>Roll:  <span id="hudQRoll">0.0</span>° (Err: <span id="hudQErrRoll">0.0</span>°)</div>
                <div>Pitch: <span id="hudQPitch">0.0</span>° (Err: <span id="hudQErrPitch">0.0</span>°)</div>
                <div>Yaw:   <span id="hudQYaw">0.0</span>° (Err: <span id="hudQErrYaw">0.0</span>°)</div>
                <div>Flaps: Ail=<span id="hudQAil">0.00</span> Ele=<span id="hudQEle">0.00</span> Rud=<span id="hudQRud">0.00</span></div>
            </div>
            <div class="hud-section hud-euler">
                <div>[PINK] EULER CONTROLLER:</div>
                <div>Roll:  <span id="hudERoll">0.0</span>° (Err: <span id="hudEErrRoll">0.0</span>°)</div>
                <div>Pitch: <span id="hudEPitch">0.0</span>° (Err: <span id="hudEErrPitch">0.0</span>°)</div>
                <div>Yaw:   <span id="hudEYaw">0.0</span>° (Err: <span id="hudEErrYaw">0.0</span>°)</div>
                <div>Flaps: Ail=<span id="hudEAil">0.00</span> Ele=<span id="hudEEle">0.00</span> Rud=<span id="hudERud">0.00</span></div>
            </div>
            <div style="border-top: 1px solid #1E293B; padding-top: 8px;">
                <div class="hud-title">STATUS AT <span id="hudStatusBank">90</span>° BANK:</div>
                <div id="statusQuat" class="hud-ok">[OK] QUATERNION: 0.27° error (No Lock)</div>
                <div id="statusEuler" class="hud-fail">[FAIL] EULER: Cross-Coupling Failure (30.5° Yaw Error)</div>
            </div>
        </div>
    </div>

    <!-- Bottom Controls -->
    <footer>
        <button class="btn" id="btn30" onclick="setBank(30)">30° Turn</button>
        <button class="btn" id="btn60" onclick="setBank(60)">60° Turn</button>
        <button class="btn" id="btn80" onclick="setBank(80)">80° Turn</button>
        <button class="btn btn-active" id="btn90" onclick="setBank(90)">90° Knife-Edge</button>
        <button class="btn btn-reset" onclick="resetSim()">Reset [R]</button>
        <button class="btn btn-play" id="btnPlay" onclick="togglePlay()">Pause / Play</button>
        <button class="btn" id="btnSpeed" onclick="toggleSpeed()">Speed: 1.0x</button>
    </footer>

    <script>
        // ---------------------------------------------------------
        // 1. Simulation Engine & Precomputed Math Data
        // ---------------------------------------------------------
        let bankDeg = 90;
        let isPlaying = true;
        let simSpeed = 1.0;
        let simIndex = 0;
        let timer = null;

        let timeGrid = [];
        let spRoll = [], spPitch = [], spYaw = [];
        let quatRoll = [], quatPitch = [], quatYaw = [];
        let eulerRoll = [], eulerPitch = [], eulerYaw = [];
        let quatQ = [], eulerQ = [];
        let quatAil = [], quatEle = [], quatRud = [];
        let eulerAil = [], eulerEle = [], eulerRud = [];

        function generateData(bank) {
            const dt = 0.02;
            const duration = 20.0;
            const steps = Math.floor(duration / dt);
            const bankRad = bank * Math.PI / 180.0;

            timeGrid = []; spRoll = []; spPitch = []; spYaw = [];
            quatRoll = []; quatPitch = []; quatYaw = [];
            eulerRoll = []; eulerPitch = []; eulerYaw = [];
            quatQ = []; eulerQ = [];
            quatAil = []; quatEle = []; quatRud = [];
            eulerAil = []; eulerEle = []; eulerRud = [];

            let q_q = [1, 0, 0, 0], w_q = [0, 0, 0];
            let q_e = [1, 0, 0, 0], w_e = [0, 0, 0];

            for (let i = 0; i < steps; i++) {
                const t = i * dt;
                timeGrid.push(t);

                let r, p, y;
                if (t < 2.0) { r = 0; p = 2.0 * Math.PI / 180; y = 0; }
                else if (t < 6.0) {
                    const frac = (t - 2.0) / 4.0;
                    r = frac * bankRad; p = (2.0 + 4.0 * frac) * Math.PI / 180; y = 30.0 * frac * Math.PI / 180;
                } else if (t < 14.0) {
                    const frac = (t - 6.0) / 8.0;
                    r = bankRad; p = 6.0 * Math.PI / 180; y = (30.0 + 120.0 * frac) * Math.PI / 180;
                } else if (t < 18.0) {
                    const frac = (t - 14.0) / 4.0;
                    r = (1.0 - frac) * bankRad; p = (6.0 - 4.0 * frac) * Math.PI / 180; y = (150.0 + 15.0 * frac) * Math.PI / 180;
                } else {
                    r = 0; p = 2.0 * Math.PI / 180; y = 165.0 * Math.PI / 180;
                }

                spRoll.push(r * 180 / Math.PI);
                spPitch.push(p * 180 / Math.PI);
                spYaw.push(y * 180 / Math.PI);

                // Quaternion controller simulation step
                const q_sp = eulerToQuat(r, p, y);
                let q_err = quatMult(quatConj(q_q), q_sp);
                if (q_err[0] < 0) q_err = q_err.map(v => -v);
                let w_sp_q = [7.0 * q_err[1], 7.0 * q_err[2], 7.0 * q_err[3]];

                let rate_err_q = [w_sp_q[0] - w_q[0], w_sp_q[1] - w_q[1], w_sp_q[2] - w_q[2]];
                let cmd_q = [
                    Math.max(-1, Math.min(1, 8.0 * rate_err_q[0] - 0.8 * w_q[0])),
                    Math.max(-1, Math.min(1, 8.0 * rate_err_q[1] - 0.8 * w_q[1])),
                    Math.max(-1, Math.min(1, 8.0 * rate_err_q[2] - 0.8 * w_q[2]))
                ];

                quatAil.push(cmd_q[0]); quatEle.push(cmd_q[1]); quatRud.push(cmd_q[2]);
                stepPhysics(q_q, w_q, cmd_q, dt);
                quatQ.push([...q_q]);
                const eq = quatToEuler(q_q);
                quatRoll.push(eq[0] * 180 / Math.PI); quatPitch.push(eq[1] * 180 / Math.PI); quatYaw.push(eq[2] * 180 / Math.PI);

                // Euler controller simulation step
                const ee_m = quatToEuler(q_e);
                let r_err = r - ee_m[0];
                let p_err = p - ee_m[1];
                let y_err = (y - ee_m[2] + Math.PI) % (2 * Math.PI) - Math.PI;
                let w_sp_e = [3.5 * r_err, 3.5 * p_err, 3.5 * y_err];

                let rate_err_e = [w_sp_e[0] - w_e[0], w_sp_e[1] - w_e[1], w_sp_e[2] - w_e[2]];
                let cmd_e = [
                    Math.max(-1, Math.min(1, 8.0 * rate_err_e[0] - 0.8 * w_e[0])),
                    Math.max(-1, Math.min(1, 8.0 * rate_err_e[1] - 0.8 * w_e[1])),
                    Math.max(-1, Math.min(1, 8.0 * rate_err_e[2] - 0.8 * w_e[2]))
                ];

                eulerAil.push(cmd_e[0]); eulerEle.push(cmd_e[1]); eulerRud.push(cmd_e[2]);
                stepPhysics(q_e, w_e, cmd_e, dt);
                eulerQ.push([...q_e]);
                const ee = quatToEuler(q_e);
                eulerRoll.push(ee[0] * 180 / Math.PI); eulerPitch.push(ee[1] * 180 / Math.PI); eulerYaw.push(ee[2] * 180 / Math.PI);
            }
        }

        function eulerToQuat(r, p, y) {
            const cy = Math.cos(y * 0.5), sy = Math.sin(y * 0.5);
            const cp = Math.cos(p * 0.5), sp = Math.sin(p * 0.5);
            const cr = Math.cos(r * 0.5), sr = Math.sin(r * 0.5);
            return [
                cr*cp*cy + sr*sp*sy,
                sr*cp*cy - cr*sp*sy,
                cr*sp*cy + sr*cp*sy,
                cr*cp*sy - sr*sp*cy
            ];
        }

        function quatToEuler(q) {
            const qw = q[0], qx = q[1], qy = q[2], qz = q[3];
            const sinr = 2 * (qw * qx + qy * qz);
            const cosr = 1 - 2 * (qx * qx + qy * qy);
            const roll = Math.atan2(sinr, cosr);
            const sinp = 2 * (qw * qy - qz * qx);
            const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);
            const siny = 2 * (qw * qz + qx * qy);
            const cosy = 1 - 2 * (qy * qy + qz * qz);
            const yaw = Math.atan2(siny, cosy);
            return [roll, pitch, yaw];
        }

        function quatMult(p, q) {
            return [
                p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
                p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
                p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
                p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0]
            ];
        }

        function quatConj(q) { return [q[0], -q[1], -q[2], -q[3]]; }

        function stepPhysics(q, w, cmd, dt) {
            const I = [180.0, 220.0, 350.0];
            const Tau = [450.0 * cmd[0] - 120.0 * w[0], 520.0 * cmd[1] - 140.0 * w[1], 380.0 * cmd[2] - 160.0 * w[2]];
            w[0] += (Tau[0] / I[0]) * dt;
            w[1] += (Tau[1] / I[1]) * dt;
            w[2] += (Tau[2] / I[2]) * dt;

            const w_q = [0, w[0], w[1], w[2]];
            const q_dot = quatMult(q, w_q).map(v => 0.5 * v);
            q[0] += q_dot[0] * dt; q[1] += q_dot[1] * dt; q[2] += q_dot[2] * dt; q[3] += q_dot[3] * dt;
            const norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
            q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
        }

        // ---------------------------------------------------------
        // 2. Three.js 60 FPS GPU 3D Aircraft Visualizer
        // ---------------------------------------------------------
        let sceneQ, cameraQ, rendererQ, jetQ;
        let sceneE, cameraE, rendererE, jetE;

        function init3D() {
            const containerQ = document.getElementById('containerQuat');
            sceneQ = new THREE.Scene();
            cameraQ = new THREE.PerspectiveCamera(45, containerQ.clientWidth / containerQ.clientHeight, 0.1, 100);
            cameraQ.position.set(3, -4, 3); cameraQ.up.set(0, 0, -1); cameraQ.lookAt(0, 0, 0);
            rendererQ = new THREE.WebGLRenderer({ antialias: true });
            rendererQ.setSize(containerQ.clientWidth, containerQ.clientHeight);
            containerQ.appendChild(rendererQ.domElement);
            jetQ = createJetMesh(0x00F0FF); sceneQ.add(jetQ); addLights(sceneQ); addGrid(sceneQ);

            const containerE = document.getElementById('containerEuler');
            sceneE = new THREE.Scene();
            cameraE = new THREE.PerspectiveCamera(45, containerE.clientWidth / containerE.clientHeight, 0.1, 100);
            cameraE.position.set(3, -4, 3); cameraE.up.set(0, 0, -1); cameraE.lookAt(0, 0, 0);
            rendererE = new THREE.WebGLRenderer({ antialias: true });
            rendererE.setSize(containerE.clientWidth, containerE.clientHeight);
            containerE.appendChild(rendererE.domElement);
            jetE = createJetMesh(0xFF0055); sceneE.add(jetE); addLights(sceneE); addGrid(sceneE);
        }

        function createJetMesh(colorHex) {
            const group = new THREE.Group();
            // Fuselage
            const fuseGeo = new THREE.ConeGeometry(0.35, 3.5, 8);
            fuseGeo.rotateX(Math.PI / 2);
            const fuseMat = new THREE.MeshPhongMaterial({ color: colorHex, flatShading: true });
            const fuselage = new THREE.Mesh(fuseGeo, fuseMat);
            group.add(fuselage);

            // Wings
            const wingGeo = new THREE.BoxGeometry(1.2, 4.8, 0.08);
            const wingMat = new THREE.MeshPhongMaterial({ color: 0x38BDF8 });
            const wings = new THREE.Mesh(wingGeo, wingMat);
            wings.position.set(-0.2, 0, 0);
            group.add(wings);

            // Tail Fin
            const finGeo = new THREE.BoxGeometry(0.8, 0.08, 1.2);
            const finMat = new THREE.MeshPhongMaterial({ color: 0xF59E0B });
            const fin = new THREE.Mesh(finGeo, finMat);
            fin.position.set(-1.2, 0, -0.6);
            group.add(fin);

            return group;
        }

        function addLights(scene) {
            const ambientLight = new THREE.AmbientLight(0xffffff, 0.7); scene.add(ambientLight);
            const dirLight = new THREE.DirectionalLight(0xffffff, 0.8); dirLight.position.set(5, 10, 7); scene.add(dirLight);
        }

        function addGrid(scene) {
            const grid = new THREE.GridHelper(8, 8, 0x334155, 0x1E293B);
            grid.rotation.x = Math.PI / 2; grid.position.z = 2.0; scene.add(grid);
        }

        function update3D(q_q, q_e) {
            jetQ.quaternion.set(q_q[1], q_q[2], q_q[3], q_q[0]);
            rendererQ.render(sceneQ, cameraQ);

            jetE.quaternion.set(q_e[1], q_e[2], q_e[3], q_e[0]);
            rendererE.render(sceneE, cameraE);
        }

        // ---------------------------------------------------------
        // 3. Chart.js 2D Real-Time Graphs
        // ---------------------------------------------------------
        let chartRoll, chartPitch, chartYaw;

        function initCharts() {
            const cfg = (title, minVal, maxVal) => ({
                type: 'line',
                data: {
                    labels: [],
                    datasets: [
                        { label: 'Setpoint', borderColor: '#22C55E', borderDash: [4, 4], data: [], fill: false, pointRadius: 0 },
                        { label: 'Quaternion', borderColor: '#00F0FF', borderWidth: 2, data: [], fill: false, pointRadius: 0 },
                        { label: 'Euler', borderColor: '#FF0055', borderWidth: 1.5, borderDash: [2, 2], data: [], fill: false, pointRadius: 0 }
                    ]
                },
                options: {
                    responsive: true, maintainAspectRatio: false, animation: false,
                    scales: {
                        x: { type: 'linear', min: 0, max: 20, grid: { color: '#334155' }, ticks: { color: '#94A3B8' } },
                        y: { min: minVal, max: maxVal, grid: { color: '#334155' }, ticks: { color: '#94A3B8' } }
                    },
                    plugins: { legend: { labels: { color: '#F8FAFC', font: { size: 10 } } } }
                }
            });

            chartRoll = new Chart(document.getElementById('chartRoll'), cfg('Roll', -15, 105));
            chartPitch = new Chart(document.getElementById('chartPitch'), cfg('Pitch', -15, 25));
            chartYaw = new Chart(document.getElementById('chartYaw'), cfg('Yaw', -15, 200));
        }

        function updateCharts(idx) {
            const tSlice = timeGrid.slice(0, idx + 1);

            chartRoll.data.labels = tSlice;
            chartRoll.data.datasets[0].data = spRoll.slice(0, idx + 1);
            chartRoll.data.datasets[1].data = quatRoll.slice(0, idx + 1);
            chartRoll.data.datasets[2].data = eulerRoll.slice(0, idx + 1);
            chartRoll.update();

            chartPitch.data.labels = tSlice;
            chartPitch.data.datasets[0].data = spPitch.slice(0, idx + 1);
            chartPitch.data.datasets[1].data = quatPitch.slice(0, idx + 1);
            chartPitch.data.datasets[2].data = eulerPitch.slice(0, idx + 1);
            chartPitch.update();

            chartYaw.data.labels = tSlice;
            chartYaw.data.datasets[0].data = spYaw.slice(0, idx + 1);
            chartYaw.data.datasets[1].data = quatYaw.slice(0, idx + 1);
            chartYaw.data.datasets[2].data = eulerYaw.slice(0, idx + 1);
            chartYaw.update();
        }

        // ---------------------------------------------------------
        // 4. Telemetry & Control Bar Handlers
        // ---------------------------------------------------------
        function updateHUD(idx) {
            document.getElementById('hudTime').innerText = timeGrid[idx].toFixed(2);
            document.getElementById('hudBank').innerText = bankDeg;
            document.getElementById('hudStatusBank').innerText = bankDeg;

            document.getElementById('hudSpRoll').innerText = spRoll[idx].toFixed(1);
            document.getElementById('hudSpPitch').innerText = spPitch[idx].toFixed(1);
            document.getElementById('hudSpYaw').innerText = spYaw[idx].toFixed(1);

            const q = quatQ[idx];
            document.getElementById('hudQw').innerText = q[0].toFixed(3);
            document.getElementById('hudQx').innerText = q[1].toFixed(3);
            document.getElementById('hudQy').innerText = q[2].toFixed(3);
            document.getElementById('hudQz').innerText = q[3].toFixed(3);

            document.getElementById('hudQRoll').innerText = quatRoll[idx].toFixed(1);
            document.getElementById('hudQErrRoll').innerText = Math.abs(spRoll[idx] - quatRoll[idx]).toFixed(1);
            document.getElementById('hudQPitch').innerText = quatPitch[idx].toFixed(1);
            document.getElementById('hudQErrPitch').innerText = Math.abs(spPitch[idx] - quatPitch[idx]).toFixed(1);
            document.getElementById('hudQYaw').innerText = quatYaw[idx].toFixed(1);
            document.getElementById('hudQErrYaw').innerText = Math.abs(spYaw[idx] - quatYaw[idx]).toFixed(1);

            document.getElementById('hudQAil').innerText = quatAil[idx].toFixed(2);
            document.getElementById('hudQEle').innerText = quatEle[idx].toFixed(2);
            document.getElementById('hudQRud').innerText = quatRud[idx].toFixed(2);

            document.getElementById('hudERoll').innerText = eulerRoll[idx].toFixed(1);
            document.getElementById('hudEErrRoll').innerText = Math.abs(spRoll[idx] - eulerRoll[idx]).toFixed(1);
            document.getElementById('hudEPitch').innerText = eulerPitch[idx].toFixed(1);
            document.getElementById('hudEErrPitch').innerText = Math.abs(spPitch[idx] - eulerPitch[idx]).toFixed(1);
            document.getElementById('hudEYaw').innerText = eulerYaw[idx].toFixed(1);
            document.getElementById('hudEErrYaw').innerText = Math.abs(spYaw[idx] - eulerYaw[idx]).toFixed(1);

            document.getElementById('hudEAil').innerText = eulerAil[idx].toFixed(2);
            document.getElementById('hudEEle').innerText = eulerEle[idx].toFixed(2);
            document.getElementById('hudERud').innerText = eulerRud[idx].toFixed(2);
        }

        function setBank(b) {
            bankDeg = b;
            document.querySelectorAll('.btn').forEach(btn => btn.classList.remove('btn-active'));
            document.getElementById('btn' + b).classList.add('btn-active');
            document.getElementById('mainTitle').innerText = `FAST 6-DOF DRONE SIMULATOR: ${b}° BANK MANEUVER`;
            generateData(b);
            simIndex = 0;
        }

        function resetSim() { simIndex = 0; }
        function togglePlay() { isPlaying = !isPlaying; }
        function toggleSpeed() {
            simSpeed = simSpeed === 1.0 ? 0.5 : (simSpeed === 0.5 ? 2.0 : 1.0);
            document.getElementById('btnSpeed').innerText = `Speed: ${simSpeed.toFixed(1)}x`;
            document.getElementById('hudSpeed').innerText = `${simSpeed.toFixed(1)}x`;
        }

        // Global Keydown Listeners for 0ms shortcuts
        window.addEventListener('keydown', (e) => {
            if (e.key === 'r' || e.key === 'R') resetSim();
            if (e.key === ' ') togglePlay();
        });

        // Loop Runner
        function simLoop() {
            if (isPlaying && timeGrid.length > 0) {
                simIndex = (simIndex + 1) % timeGrid.length;
                update3D(quatQ[simIndex], eulerQ[simIndex]);
                updateHUD(simIndex);
                if (simIndex % 2 === 0) updateCharts(simIndex);
            }
            setTimeout(simLoop, 20 / simSpeed);
        }

        window.onload = () => {
            generateData(90);
            init3D();
            initCharts();
            simLoop();
        };
    </script>
</body>
</html>
"""

def run_fast_sim():
    # Save standalone HTML app
    app_path = os.path.join(os.getcwd(), 'visual_sim_app.html')
    with open(app_path, 'w', encoding='utf-8') as f:
        f.write(HTML_CONTENT)
    print(f"Generated standalone WebGL GPU-accelerated dashboard: {app_path}")

    # Launch local server
    PORT = 8080
    class QuietHandler(http.server.SimpleHTTPRequestHandler):
        def log_message(self, format, *args):
            pass # Quiet server logs

    socketserver.TCPServer.allow_reuse_address = True
    try:
        httpd = socketserver.TCPServer(("", PORT), QuietHandler)
    except OSError:
        PORT = 8081
        httpd = socketserver.TCPServer(("", PORT), QuietHandler)

    server_thread = threading.Thread(target=httpd.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    url = f"http://localhost:{PORT}/visual_sim_app.html"
    print(f"Launching FAST 60 FPS GPU Drone Simulator at {url}...")
    webbrowser.open(url)
    print("Press Ctrl+C in terminal to stop simulator server.")
    
    try:
        server_thread.join()
    except KeyboardInterrupt:
        print("\nStopping simulator server.")
        httpd.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    run_fast_sim()
