import streamlit as st
import numpy as np
import pandas as pd
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import time
import math

# Page configuration
st.set_page_config(
    page_title="Sugar Rocket Trajectory Simulator",
    page_icon="🚀",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better styling
st.markdown("""
<style>
    .main-header {
        background: linear-gradient(90deg, #4A90E2, #7B68EE);
        -webkit-background-clip: text;
        -webkit-text-fill-color: transparent;
        font-size: 2.5rem;
        font-weight: bold;
        text-align: center;
        margin-bottom: 1rem;
    }
    .metric-card {
        background-color: #1E3A8A;
        padding: 1rem;
        border-radius: 0.5rem;
        border: 1px solid #3B82F6;
    }
</style>
""", unsafe_allow_html=True)

class RocketSimulator:
    def __init__(self, params):
        self.params = params
        self.reset_simulation()
    
    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.time = 0
        self.state = {
            'x': 0, 'y': self.params['launch_height'],
            'vx': 0, 'vy': 0,
            'ax': 0, 'ay': 0,
            'mass': self.params['dry_mass'] + self.params['propellant_mass'],
            'thrust': 0, 'drag': 0,
            'phase': 'powered',
            'max_altitude': 0, 'max_velocity': 0,
            'burnout_time': 0, 'apogee_time': 0
        }
        self.trajectory = []
    
    def get_thrust_at_time(self, t):
        """Calculate thrust based on time and thrust curve type"""
        if t > self.params['burn_time']:
            return 0
        
        avg_thrust = self.params['total_impulse'] / self.params['burn_time']
        progress = t / self.params['burn_time']
        
        if self.params['thrust_curve_type'] == 'progressive':
            return avg_thrust * (0.3 + 1.4 * progress)
        elif self.params['thrust_curve_type'] == 'regressive':
            return avg_thrust * (1.7 - 1.4 * progress)
        else:  # neutral
            return avg_thrust
    
    def get_air_density(self, altitude):
        """Calculate air density at given altitude"""
        scale_height = 8400  # meters
        return self.params['air_density'] * math.exp(-altitude / scale_height)
    
    def calculate_drag(self, velocity_x, velocity_y, altitude, area, drag_coeff):
        """Calculate drag force components"""
        rho = self.get_air_density(altitude)
        v_mag = math.sqrt(velocity_x**2 + velocity_y**2)
        
        if v_mag == 0:
            return 0, 0
        
        drag_mag = 0.5 * rho * v_mag**2 * area * drag_coeff
        
        return (-drag_mag * velocity_x / v_mag, -drag_mag * velocity_y / v_mag)
    
    def simulate_step(self, dt):
        """Simulate one time step"""
        # Calculate current mass
        if self.time <= self.params['burn_time']:
            burn_progress = self.time / self.params['burn_time']
            self.state['mass'] = (self.params['dry_mass'] + 
                                self.params['propellant_mass'] * (1 - burn_progress))
        else:
            self.state['mass'] = self.params['dry_mass']
        
        # Calculate thrust
        self.state['thrust'] = self.get_thrust_at_time(self.time)
        
        # Reference area for drag
        area = math.pi * (self.params['diameter'] / 2)**2
        
        # Determine flight phase
        if self.time <= self.params['burn_time']:
            self.state['phase'] = 'powered'
        elif self.state['vy'] > 0:
            self.state['phase'] = 'ballistic'
        elif self.state['y'] > self.params['recovery_deploy_altitude']:
            self.state['phase'] = 'recovery'
        else:
            self.state['phase'] = 'landed'
        
        # Calculate forces
        launch_angle_rad = math.radians(self.params['launch_angle'])
        
        # Thrust force components
        thrust_x = thrust_y = 0
        if self.state['phase'] == 'powered':
            thrust_x = self.state['thrust'] * math.cos(launch_angle_rad)
            thrust_y = self.state['thrust'] * math.sin(launch_angle_rad)
        
        # Drag force
        drag_coeff = self.params['drag_coefficient']
        drag_area = area
        
        if self.state['phase'] == 'recovery':
            drag_coeff = self.params['recovery_drag_coeff']
            drag_area = self.params['recovery_area']
        
        drag_x, drag_y = self.calculate_drag(
            self.state['vx'], self.state['vy'], self.state['y'], drag_area, drag_coeff
        )
        
        # Wind effect
        wind_drag = -0.1 * (self.state['vx'] - self.params['wind_speed'])
        
        # Net acceleration
        self.state['ax'] = (thrust_x + drag_x + wind_drag) / self.state['mass']
        self.state['ay'] = (thrust_y + drag_y - 
                          self.params['gravity'] * self.state['mass']) / self.state['mass']
        
        # Update velocity and position
        self.state['vx'] += self.state['ax'] * dt
        self.state['vy'] += self.state['ay'] * dt
        self.state['x'] += self.state['vx'] * dt
        self.state['y'] = max(0, self.state['y'] + self.state['vy'] * dt)
        
        # Track maximums
        self.state['max_altitude'] = max(self.state['max_altitude'], self.state['y'])
        velocity = math.sqrt(self.state['vx']**2 + self.state['vy']**2)
        self.state['max_velocity'] = max(self.state['max_velocity'], velocity)
        
        # Track events
        if (self.time > self.params['burn_time'] and 
            self.state['burnout_time'] == 0):
            self.state['burnout_time'] = self.time
        
        if (self.state['vy'] < 0 and len(self.trajectory) > 0 and 
            self.trajectory[-1]['vy'] >= 0 and self.state['apogee_time'] == 0):
            self.state['apogee_time'] = self.time
        
        # Record trajectory point
        self.trajectory.append({
            'time': round(self.time, 2),
            'x': round(self.state['x'], 2),
            'y': round(self.state['y'], 2),
            'vx': round(self.state['vx'], 2),
            'vy': round(self.state['vy'], 2),
            'velocity': round(velocity, 2),
            'altitude': round(self.state['y'], 2),
            'thrust': round(self.state['thrust'], 2),
            'phase': self.state['phase'],
            'mass': round(self.state['mass'], 3)
        })
        
        self.time += dt
        
        # Check if simulation should end
        return self.state['y'] > 0 and self.time < 60
    
    def run_full_simulation(self):
        """Run complete simulation and return trajectory data"""
        self.reset_simulation()
        dt = 0.01  # 10ms time steps
        
        while self.simulate_step(dt):
            pass  # Continue until landing or timeout
        
        return pd.DataFrame(self.trajectory)

def generate_matlab_script(df, params):
    """Generate MATLAB script for 3D visualization"""
    
    # Convert dataframe to MATLAB-friendly format
    matlab_code = f"""% Rocket Trajectory 3D Visualization
% Generated from Sugar Rocket Simulator
% Data contains {len(df)} trajectory points

clear; clc; close all;

%% Simulation Parameters
rocket_params.total_impulse = {params['total_impulse']};
rocket_params.burn_time = {params['burn_time']};
rocket_params.dry_mass = {params['dry_mass']};
rocket_params.propellant_mass = {params['propellant_mass']};
rocket_params.diameter = {params['diameter']};
rocket_params.length = {params['length']};
rocket_params.launch_angle = {params['launch_angle']};

%% Trajectory Data
time = [{', '.join(map(str, df['time'].tolist()))}];
x = [{', '.join(map(str, df['x'].tolist()))}];
y = [{', '.join(map(str, df['y'].tolist()))}];
z = zeros(size(x)); % Assuming 2D flight, extend to 3D as needed
velocity = [{', '.join(map(str, df['velocity'].tolist()))}];
thrust = [{', '.join(map(str, df['thrust'].tolist()))}];

%% 3D Visualization Setup
figure('Position', [100, 100, 1400, 800]);

% Create subplot layout
subplot(2,3,[1,2,4,5]); % Large 3D plot

%% 3D Trajectory Plot
plot3(x, z, y, 'LineWidth', 3, 'Color', [0.2, 0.6, 0.8]);
hold on;

% Mark key points
[max_alt, max_idx] = max(y);
plot3(x(max_idx), z(max_idx), y(max_idx), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'red');
plot3(x(1), z(1), y(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'green');
plot3(x(end), z(end), y(end), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'black');

% Ground plane
[X_ground, Z_ground] = meshgrid(min(x)-50:20:max(x)+50, -100:20:100);
Y_ground = zeros(size(X_ground));
surf(X_ground, Z_ground, Y_ground, 'FaceAlpha', 0.3, 'FaceColor', [0.8, 0.6, 0.4]);

% Labels and formatting
xlabel('Horizontal Distance (m)', 'FontSize', 12);
ylabel('Cross-range (m)', 'FontSize', 12);
zlabel('Altitude (m)', 'FontSize', 12);
title('3D Rocket Trajectory Visualization', 'FontSize', 16, 'FontWeight', 'bold');
grid on;
axis equal;
view(45, 30);

% Legend
legend({{'Trajectory', 'Apogee', 'Launch', 'Landing', 'Ground'}}, 'Location', 'best');

%% Side view plots
subplot(2,3,3);
plot(x, y, 'LineWidth', 2, 'Color', [0.8, 0.2, 0.2]);
xlabel('Horizontal Distance (m)');
ylabel('Altitude (m)');
title('Side View');
grid on;

subplot(2,3,6);
plot(time, velocity, 'LineWidth', 2, 'Color', [0.2, 0.8, 0.2]);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Profile');
grid on;

%% Animation Function
function animate_rocket()
    figure('Position', [200, 200, 1000, 700]);
    
    % Setup 3D plot
    plot3(x, z, y, '--', 'Color', [0.7, 0.7, 0.7], 'LineWidth', 1);
    hold on;
    
    % Ground plane
    [X_ground, Z_ground] = meshgrid(min(x)-50:50:max(x)+50, -100:50:100);
    Y_ground = zeros(size(X_ground));
    surf(X_ground, Z_ground, Y_ground, 'FaceAlpha', 0.2, 'FaceColor', [0.6, 0.4, 0.2]);
    
    xlabel('Horizontal Distance (m)');
    ylabel('Cross-range (m)');
    zlabel('Altitude (m)');
    title('Animated Rocket Flight');
    grid on;
    axis equal;
    view(45, 30);
    
    % Rocket representation
    rocket_trail = plot3([], [], [], 'r-', 'LineWidth', 3);
    rocket_point = plot3([], [], [], 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
    
    % Animation loop
    for i = 1:5:length(time) % Skip frames for speed
        % Update rocket position
        set(rocket_trail, 'XData', x(1:i), 'YData', z(1:i), 'ZData', y(1:i));
        set(rocket_point, 'XData', x(i), 'YData', z(i), 'ZData', y(i));
        
        % Update title with current data
        title(sprintf('Rocket Flight - T: %.1fs, Alt: %.1fm, Vel: %.1fm/s', ...
              time(i), y(i), velocity(i)));
        
        % Camera following (optional)
        if y(i) > 50
            xlim([x(i)-200, x(i)+200]);
            zlim([0, max(y)+100]);
        end
        
        pause(0.05); % Animation speed
        drawnow;
    end
end

%% Enhanced 3D Rocket Model
function draw_rocket_3d(position, orientation, scale)
    % Draw a simple 3D rocket model at given position
    % position: [x, y, z]
    % orientation: [pitch, yaw, roll] in degrees
    % scale: size multiplier
    
    if nargin < 3, scale = 1; end
    if nargin < 2, orientation = [90, 0, 0]; end
    
    % Basic rocket geometry (cylinder + cone)
    [X_body, Y_body, Z_body] = cylinder([1, 1], 20);
    Z_body = Z_body * 3 * scale; % Body length
    
    % Nose cone
    [X_nose, Y_nose, Z_nose] = cylinder([1, 0], 10);
    Z_nose = Z_nose * scale + 3 * scale; % Nose length
    
    % Apply transformations and plot
    surf(X_body * scale + position(1), Y_body * scale + position(2), ...
         Z_body + position(3), 'FaceColor', 'white', 'EdgeColor', 'none');
    hold on;
    surf(X_nose * scale + position(1), Y_nose * scale + position(2), ...
         Z_nose + position(3), 'FaceColor', 'red', 'EdgeColor', 'none');
end

%% Performance Analysis
fprintf('\\n=== FLIGHT PERFORMANCE SUMMARY ===\\n');
fprintf('Maximum Altitude: %.1f m (%.0f ft)\\n', max(y), max(y)*3.28084);
fprintf('Maximum Velocity: %.1f m/s (%.0f mph)\\n', max(velocity), max(velocity)*2.237);
fprintf('Flight Time: %.1f seconds\\n', time(end));
fprintf('Horizontal Distance: %.1f m\\n', x(end));

% Find burnout time
burnout_idx = find(thrust > 0, 1, 'last');
if ~isempty(burnout_idx)
    fprintf('Burnout Time: %.1f seconds\\n', time(burnout_idx));
    fprintf('Burnout Altitude: %.1f m\\n', y(burnout_idx));
end

% Find apogee time
[~, apogee_idx] = max(y);
fprintf('Time to Apogee: %.1f seconds\\n', time(apogee_idx));

%% Additional Analysis Functions

% Power spectral density of acceleration
function analyze_flight_dynamics()
    % Calculate acceleration from velocity
    dt = mean(diff(time));
    accel = gradient(velocity, dt);
    
    figure;
    subplot(2,1,1);
    plot(time, accel);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s²)');
    title('Flight Acceleration Profile');
    grid on;
    
    % Frequency analysis
    subplot(2,1,2);
    [pxx, f] = pwelch(accel, [], [], [], 1/dt);
    semilogy(f, pxx);
    xlabel('Frequency (Hz)');
    ylabel('Power Spectral Density');
    title('Acceleration Frequency Content');
    grid on;
end

%% Save data in MATLAB format
save('rocket_trajectory_data.mat', 'time', 'x', 'y', 'z', 'velocity', 'thrust', 'rocket_params');

%% Run visualization
fprintf('\\nMAT-file saved as: rocket_trajectory_data.mat\\n');
fprintf('Run animate_rocket() for animation\\n');
fprintf('Run analyze_flight_dynamics() for detailed analysis\\n');

% Uncomment to run animation automatically
% animate_rocket();
"""
    
    return matlab_code


def main():
    st.markdown('<h1 class="main-header">🚀 Advanced Sugar Rocket Trajectory Simulator</h1>', 
                unsafe_allow_html=True)
    st.markdown("### High-fidelity physics simulation with atmospheric modeling")
    
    # Sidebar for parameters
    st.sidebar.header("🔧 Rocket Configuration")
    
    # Motor characteristics
    st.sidebar.subheader("Motor Characteristics")
    total_impulse = st.sidebar.slider("Total Impulse (N⋅s)", 50, 500, 160, 10)
    burn_time = st.sidebar.slider("Burn Time (s)", 1.0, 10.0, 2.8, 0.1)
    propellant_mass = st.sidebar.slider("Propellant Mass (kg)", 0.1, 1.0, 0.24, 0.01)
    thrust_curve = st.sidebar.selectbox("Thrust Curve", 
                                       ["progressive", "neutral", "regressive"])
    
    # Physical characteristics
    st.sidebar.subheader("Physical Characteristics")
    dry_mass = st.sidebar.slider("Dry Mass (kg)", 0.3, 2.0, 0.8, 0.1)
    diameter = st.sidebar.slider("Diameter (mm)", 30, 100, 54, 1) / 1000  # Convert to meters
    length = st.sidebar.slider("Length (m)", 0.3, 1.5, 0.6, 0.1)
    drag_coefficient = st.sidebar.slider("Drag Coefficient", 0.3, 0.8, 0.45, 0.05)
    
    # Launch parameters
    st.sidebar.subheader("Launch Parameters")
    launch_angle = st.sidebar.slider("Launch Angle (°)", 45, 90, 85, 1)
    launch_height = st.sidebar.slider("Launch Height (m)", 0, 50, 0, 1)
    
    # Environmental parameters
    st.sidebar.subheader("Environmental Conditions")
    air_density = st.sidebar.slider("Air Density (kg/m³)", 1.0, 1.3, 1.225, 0.001)
    wind_speed = st.sidebar.slider("Wind Speed (m/s)", 0, 10, 2, 1)
    
    # Recovery parameters
    st.sidebar.subheader("Recovery System")
    recovery_deploy_altitude = st.sidebar.slider("Parachute Deploy Altitude (m)", 50, 300, 150, 10)
    recovery_drag_coeff = st.sidebar.slider("Parachute Drag Coefficient", 1.0, 2.0, 1.3, 0.1)
    recovery_area = st.sidebar.slider("Parachute Area (m²)", 0.1, 2.0, 0.5, 0.1)
    
    # Create parameters dictionary
    params = {
        'total_impulse': total_impulse,
        'burn_time': burn_time,
        'propellant_mass': propellant_mass,
        'dry_mass': dry_mass,
        'diameter': diameter,
        'length': length,
        'drag_coefficient': drag_coefficient,
        'launch_angle': launch_angle,
        'launch_height': launch_height,
        'air_density': air_density,
        'gravity': 9.81,
        'wind_speed': wind_speed,
        'thrust_curve_type': thrust_curve,
        'recovery_deploy_altitude': recovery_deploy_altitude,
        'recovery_drag_coeff': recovery_drag_coeff,
        'recovery_area': recovery_area
    }
    
    # Main content
    col1, col2 = st.columns([3, 1])
    
    with col2:
        st.subheader("🎮 Controls")
        
        if st.button("🚀 Run Simulation", type="primary", use_container_width=True):
            # Create and run simulation
            simulator = RocketSimulator(params)
            
            # Progress bar
            progress_bar = st.progress(0)
            status_text = st.empty()
            
            with st.spinner("Running simulation..."):
                df = simulator.run_full_simulation()
                progress_bar.progress(100)
                status_text.text("Simulation complete!")
            
            # Store results in session state
            st.session_state.simulation_data = df
            st.session_state.simulator = simulator
    
    with col1:
        # Display results if simulation has been run
        if 'simulation_data' in st.session_state:
            df = st.session_state.simulation_data
            sim = st.session_state.simulator
            
            # Key metrics
            st.subheader("📈 Flight Performance")
            
            metrics_col1, metrics_col2, metrics_col3, metrics_col4 = st.columns(4)
            
            with metrics_col1:
                st.metric(
                    "Max Altitude", 
                    f"{sim.state['max_altitude']:.1f} m",
                    f"{sim.state['max_altitude']*3.28084:.0f} ft"
                )
            
            with metrics_col2:
                st.metric(
                    "Max Velocity", 
                    f"{sim.state['max_velocity']:.1f} m/s",
                    f"{sim.state['max_velocity']*2.237:.0f} mph"
                )
            
            with metrics_col3:
                flight_time = df['time'].iloc[-1] if len(df) > 0 else 0
                st.metric("Flight Time", f"{flight_time:.1f} s")
            
            with metrics_col4:
                if sim.state['apogee_time'] > 0:
                    st.metric("Time to Apogee", f"{sim.state['apogee_time']:.1f} s")
                else:
                    st.metric("Time to Apogee", "N/A")
            
            # Trajectory plots
            st.subheader("📊 Trajectory Analysis")
            
            # Create subplots
            fig = make_subplots(
                rows=2, cols=2,
                subplot_titles=('Altitude vs Time', 'Velocity vs Time', 
                              '2D Trajectory', 'Thrust vs Time'),
                specs=[[{"secondary_y": False}, {"secondary_y": False}],
                       [{"secondary_y": False}, {"secondary_y": False}]]
            )
            
            # Altitude vs Time
            fig.add_trace(
                go.Scatter(x=df['time'], y=df['altitude'], name='Altitude',
                          line=dict(color='#4A90E2', width=3)),
                row=1, col=1
            )
            
            # Velocity vs Time
            fig.add_trace(
                go.Scatter(x=df['time'], y=df['velocity'], name='Velocity',
                          line=dict(color='#E24A4A', width=3)),
                row=1, col=2
            )
            
            # 2D Trajectory
            fig.add_trace(
                go.Scatter(x=df['x'], y=df['altitude'], name='Flight Path',
                          line=dict(color='#50C878', width=3)),
                row=2, col=1
            )
            
            # Thrust vs Time
            fig.add_trace(
                go.Scatter(x=df['time'], y=df['thrust'], name='Thrust',
                          line=dict(color='#FF6B35', width=3)),
                row=2, col=2
            )
            
            # Update layout
            fig.update_layout(
                height=600,
                showlegend=False,
                title_text="Comprehensive Flight Analysis"
            )
            
            # Update axes labels
            fig.update_xaxes(title_text="Time (s)", row=1, col=1)
            fig.update_yaxes(title_text="Altitude (m)", row=1, col=1)
            fig.update_xaxes(title_text="Time (s)", row=1, col=2)
            fig.update_yaxes(title_text="Velocity (m/s)", row=1, col=2)
            fig.update_xaxes(title_text="Horizontal Distance (m)", row=2, col=1)
            fig.update_yaxes(title_text="Altitude (m)", row=2, col=1)
            fig.update_xaxes(title_text="Time (s)", row=2, col=2)
            fig.update_yaxes(title_text="Thrust (N)", row=2, col=2)
            
            st.plotly_chart(fig, use_container_width=True)
            
            # Additional analysis
            st.subheader("🔍 Detailed Analysis")
            
            # Flight phases analysis
            phases_df = df.groupby('phase').agg({
                'time': ['min', 'max', 'count'],
                'altitude': ['min', 'max'],
                'velocity': ['min', 'max']
            }).round(2)
            
            st.write("**Flight Phases Breakdown:**")
            st.dataframe(phases_df, use_container_width=True)
            
            # Data export section
            st.subheader("📥 Export Data")
            
            col_export1, col_export2 = st.columns(2)
            
            with col_export1:
                # CSV download
                csv = df.to_csv(index=False)
                st.download_button(
                    label="Download CSV Data",
                    data=csv,
                    file_name="rocket_trajectory.csv",
                    mime="text/csv"
                )
            
            with col_export2:
                # MATLAB script download
                matlab_script = generate_matlab_script(df, params)
                st.download_button(
                    label="Download MATLAB Script",
                    data=matlab_script,
                    file_name="rocket_simulation.m",
                    mime="text/plain"
                )
            
            # Real-time trajectory viewer
            st.subheader("🎬 Animation Controls")
            
            if st.button("🎥 Animate Flight"):
                # Create animated plot
                animation_placeholder = st.empty()
                
                # Animate the trajectory
                for i in range(0, len(df), max(1, len(df)//50)):
                    current_df = df.iloc[:i+1]
                    
                    fig_anim = go.Figure()
                    
                    # Trajectory so far
                    fig_anim.add_trace(
                        go.Scatter(
                            x=current_df['x'], 
                            y=current_df['altitude'],
                            mode='lines',
                            name='Trajectory',
                            line=dict(color='blue', width=2)
                        )
                    )
                    
                    # Current position
                    if len(current_df) > 0:
                        fig_anim.add_trace(
                            go.Scatter(
                                x=[current_df['x'].iloc[-1]], 
                                y=[current_df['altitude'].iloc[-1]],
                                mode='markers',
                                name='Rocket',
                                marker=dict(color='red', size=10, symbol='triangle-up')
                            )
                        )
                    
                    fig_anim.update_layout(
                        title=f"Flight Animation - T: {current_df['time'].iloc[-1]:.1f}s",
                        xaxis_title="Horizontal Distance (m)",
                        yaxis_title="Altitude (m)",
                        height=400,
                        showlegend=False
                    )
                    
                    animation_placeholder.plotly_chart(fig_anim, use_container_width=True)
                    time.sleep(0.1)
        
        else:
            st.info("👆 Configure your rocket parameters in the sidebar and click 'Run Simulation' to begin!")
            
            # Show example configuration
            st.subheader("📋 Example Configurations")
            
            example_configs = {
                "Small Sugar Rocket": {
                    "Total Impulse": "80 N⋅s",
                    "Burn Time": "2.0 s",
                    "Dry Mass": "0.5 kg",
                    "Expected Altitude": "~200m"
                },
                "Medium Sugar Rocket": {
                    "Total Impulse": "160 N⋅s", 
                    "Burn Time": "2.8 s",
                    "Dry Mass": "0.8 kg",
                    "Expected Altitude": "~400m"
                },
                "Large Sugar Rocket": {
                    "Total Impulse": "300 N⋅s",
                    "Burn Time": "3.5 s", 
                    "Dry Mass": "1.2 kg",
                    "Expected Altitude": "~600m"
                }
            }
            
            for config_name, config_data in example_configs.items():
                with st.expander(config_name):
                    for param, value in config_data.items():
                        st.write(f"**{param}:** {value}")

if __name__ == "__main__":
    main()
