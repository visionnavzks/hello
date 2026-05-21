import json

def generate_html():
    with open('sim_data.json', 'r') as f:
        data = json.load(f)

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>High-Reach Forklift ZVD Shaper Simulation</title>
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <style>
        body {{
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background-color: #f4f7f6;
            margin: 0;
            padding: 20px;
            color: #333;
        }}
        h1 {{
            text-align: center;
            color: #2c3e50;
        }}
        .container {{
            display: flex;
            flex-direction: column;
            gap: 20px;
            max-width: 1400px;
            margin: 0 auto;
        }}
        .controls {{
            display: flex;
            justify-content: center;
            gap: 15px;
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        button {{
            padding: 10px 20px;
            font-size: 16px;
            cursor: pointer;
            border: none;
            border-radius: 5px;
            background-color: #3498db;
            color: white;
            transition: background-color 0.3s;
        }}
        button:hover {{
            background-color: #2980b9;
        }}
        button.active {{
            background-color: #2c3e50;
        }}
        .canvas-container {{
            background: white;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            padding: 20px;
            text-align: center;
        }}
        canvas#animCanvas {{
            border: 1px solid #ddd;
            background-color: #fafafa;
            width: 100%;
            height: 400px;
        }}
        .charts-container {{
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
        }}
        .chart-box {{
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }}
        @media (max-width: 900px) {{
            .charts-container {{
                grid-template-columns: 1fr;
            }}
        }}
    </style>
</head>
<body>

<h1>High-Reach Forklift Simulation (PT1 Rigorous Physics)</h1>

<div class="container">
    <div class="controls">
        <button id="btn-trap" class="active" onclick="switchMode('trap')">Trapezoidal Velocity</button>
        <button id="btn-scurve" onclick="switchMode('scurve')">S-Curve Velocity</button>
        <button onclick="playAnimation()" style="background-color: #27ae60;">Play Animation</button>
    </div>

    <div class="canvas-container">
        <h3>Forklift Animation (Red: Traditional, Blue: ZVD Shaped)</h3>
        <canvas id="animCanvas" width="1200" height="400"></canvas>
    </div>

    <div class="charts-container">
        <div class="chart-box">
            <canvas id="accelChart"></canvas>
        </div>
        <div class="chart-box">
            <canvas id="vibChart"></canvas>
        </div>
        <div class="chart-box">
            <canvas id="velChart"></canvas>
        </div>
        <div class="chart-box">
            <canvas id="posChart"></canvas>
        </div>
    </div>
</div>

<script>
    const simData = {json.dumps(data)};
    let currentMode = 'trap';
    let animationId = null;
    let isPlaying = false;

    // Chart instances
    let accelChart, vibChart, velChart, posChart;

    function initCharts() {{
        const createChart = (ctxId, title, yLabel, isAccel = false) => {{
            const ctx = document.getElementById(ctxId).getContext('2d');
            let datasets = [];
            if (isAccel) {{
                datasets = [
                    {{
                        label: 'Trad. (Cmd)',
                        borderColor: 'rgba(231, 76, 60, 0.3)',
                        borderWidth: 2,
                        borderDash: [2, 2],
                        pointRadius: 0,
                        data: []
                    }},
                    {{
                        label: 'Trad. (Actual PT1)',
                        borderColor: '#e74c3c',
                        borderWidth: 2,
                        pointRadius: 0,
                        data: []
                    }},
                    {{
                        label: 'ZVD (Cmd)',
                        borderColor: 'rgba(52, 152, 219, 0.3)',
                        borderWidth: 2,
                        borderDash: [2, 2],
                        pointRadius: 0,
                        data: []
                    }},
                    {{
                        label: 'ZVD (Actual PT1)',
                        borderColor: '#3498db',
                        borderWidth: 2,
                        pointRadius: 0,
                        data: []
                    }}
                ];
            }} else {{
                datasets = [
                    {{
                        label: 'Traditional (Raw)',
                        borderColor: '#e74c3c',
                        borderWidth: 2,
                        borderDash: [5, 5],
                        pointRadius: 0,
                        data: []
                    }},
                    {{
                        label: 'ZVD Shaped',
                        borderColor: '#3498db',
                        borderWidth: 2,
                        pointRadius: 0,
                        data: []
                    }}
                ];
            }}

            return new Chart(ctx, {{
                type: 'line',
                data: {{
                    labels: simData.t,
                    datasets: datasets
                }},
                options: {{
                    responsive: true,
                    animation: false,
                    plugins: {{
                        title: {{ display: true, text: title, font: {{size: 16}} }}
                    }},
                    scales: {{
                        x: {{ title: {{ display: true, text: 'Time (s)' }} }},
                        y: {{ title: {{ display: true, text: yLabel }} }}
                    }}
                }}
            }});
        }};

        accelChart = createChart('accelChart', 'Acceleration Command vs. Actual Base Accel', 'Acceleration (m/s²)', true);
        vibChart = createChart('vibChart', 'Cargo Vibration (Relative Deflection)', 'Displacement (m)');
        velChart = createChart('velChart', 'Forklift Base Velocity', 'Velocity (m/s)');
        posChart = createChart('posChart', 'Forklift Base Position', 'Position (m)');

        updateCharts();
    }}

    function updateCharts() {{
        const modeData = simData[currentMode];

        accelChart.data.datasets[0].data = modeData.raw.a_cmd;
        accelChart.data.datasets[1].data = modeData.raw.a_act;
        accelChart.data.datasets[2].data = modeData.zvd.a_cmd;
        accelChart.data.datasets[3].data = modeData.zvd.a_act;
        accelChart.update();

        vibChart.data.datasets[0].data = modeData.raw.y;
        vibChart.data.datasets[1].data = modeData.zvd.y;
        vibChart.update();

        velChart.data.datasets[0].data = modeData.raw.v;
        velChart.data.datasets[1].data = modeData.zvd.v;
        velChart.update();

        posChart.data.datasets[0].data = modeData.raw.p;
        posChart.data.datasets[1].data = modeData.zvd.p;
        posChart.update();
    }}

    function switchMode(mode) {{
        currentMode = mode;
        document.getElementById('btn-trap').classList.toggle('active', mode === 'trap');
        document.getElementById('btn-scurve').classList.toggle('active', mode === 'scurve');
        updateCharts();
        resetAnimation();
    }}

    // Animation Logic
    const canvas = document.getElementById('animCanvas');
    const ctx = canvas.getContext('2d');
    let frameIdx = 0;

    function drawFrame() {{
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Draw ground
        ctx.beginPath();
        ctx.moveTo(0, 360);
        ctx.lineTo(canvas.width, 360);
        ctx.strokeStyle = '#bdc3c7';
        ctx.lineWidth = 2;
        ctx.stroke();

        const t = simData.t[frameIdx];
        const modeData = simData[currentMode];

        ctx.fillStyle = '#2c3e50';
        ctx.font = "18px Arial";
        ctx.fillText(`Time: ${{t.toFixed(2)}} s`, 20, 30);

        // Draw Traditional
        drawForkliftOverlay(modeData.raw.p[frameIdx], 200, modeData.raw.y[frameIdx], 'rgba(231, 76, 60, 0.8)', 'Traditional');
        // Draw ZVD
        drawForkliftOverlay(modeData.zvd.p[frameIdx], 200, modeData.zvd.y[frameIdx], 'rgba(52, 152, 219, 0.8)', 'ZVD');

        if (isPlaying) {{
            frameIdx++;
            if (frameIdx < simData.t.length) {{
                animationId = requestAnimationFrame(drawFrame);
            }} else {{
                isPlaying = false;
            }}
        }}
    }}

    function drawForkliftOverlay(x, baseY, deflection, color, label) {{
        const scale = 80;
        const baseX = 100 + x * scale;

        ctx.fillStyle = color;
        ctx.fillRect(baseX - 40, baseY - 30, 80, 30);

        ctx.fillStyle = 'rgba(50,50,50,0.8)';
        ctx.beginPath(); ctx.arc(baseX - 25, baseY, 10, 0, Math.PI*2); ctx.fill();
        ctx.beginPath(); ctx.arc(baseX + 25, baseY, 10, 0, Math.PI*2); ctx.fill();

        const mastHeight = 150;
        ctx.fillStyle = color;
        ctx.fillRect(baseX + 20, baseY - 30 - mastHeight, 6, mastHeight);

        // Exaggerating deflection slightly for animation visibility
        const cargoX = baseX + 20 + (deflection * scale * 2);
        const cargoY = baseY - 30 - mastHeight;

        ctx.beginPath();
        ctx.moveTo(baseX + 23, cargoY);
        ctx.lineTo(cargoX + 5, cargoY + 20);
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.fillStyle = color;
        ctx.fillRect(cargoX - 15, cargoY + 20, 30, 30);

        ctx.fillStyle = 'white';
        ctx.font = "12px Arial";
        ctx.fillText(label, baseX - 30, baseY - 10);
    }}

    function playAnimation() {{
        if (!isPlaying) {{
            if (frameIdx >= simData.t.length - 1) frameIdx = 0;
            isPlaying = true;
            drawFrame();
        }}
    }}

    function resetAnimation() {{
        isPlaying = false;
        if (animationId) cancelAnimationFrame(animationId);
        frameIdx = 0;
        drawFrame();
    }}

    window.onload = () => {{
        initCharts();
        drawFrame();
    }};

</script>
</body>
</html>
"""
    with open('forklift_simulation.html', 'w') as f:
        f.write(html_content)
    print("Web simulation generated successfully as forklift_simulation.html")

if __name__ == "__main__":
    generate_html()
