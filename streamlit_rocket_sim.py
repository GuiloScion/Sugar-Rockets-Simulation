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
        
        if st.button("📊 Export Data", use_container_width=True):
            if 'simulation_data' in st.session_state:
                csv = st.session_state.simulation_data.to_csv(index=False)
                st.download_button(
                    label="Download CSV",
                    data=csv,
                    file_name="rocket_trajectory.csv",
                    mime="text/csv",
                    use_container_width=True
                )
    
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
                          line=dict(color='#E74C3C', width=3)),
                row=1, col=2
            )
            
            # 2D Trajectory
            fig.add_trace(
                go.Scatter(x=df['x'], y=df['y'], name='Trajectory',
                          line=dict(color='#2ECC71', width=3),
                          mode='lines+markers', marker=dict(size=2)),
                row=2, col=1
            )
            
            # Thrust vs Time
            fig.add_trace(
                go.Scatter(x=df['time'], y=df['thrust'], name='Thrust',
                          line=dict(color='#F39C12', width=3)),
                row=2, col=2
            )
            
            # Update layout
            fig.update_xaxes(title_text="Time (s)", row=1, col=1)
            fig.update_yaxes(title_text="Altitude (m)", row=1, col=1)
            fig.update_xaxes(title_text="Time (s)", row=1, col=2)
            fig.update_yaxes(title_text="Velocity (m/s)", row=1, col=2)
            fig.update_xaxes(title_text="Horizontal Distance (m)", row=2, col=1)
            fig.update_yaxes(title_text="Altitude (m)", row=2, col=1)
            fig.update_xaxes(title_text="Time (s)", row=2, col=2)
            fig.update_yaxes(title_text="Thrust (N)", row=2, col=2)
            
            fig.update_layout(
                height=800,
                showlegend=False,
                title_text="Rocket Flight Analysis",
                title_x=0.5
            )
            
            st.plotly_chart(fig, use_container_width=True)
            
            # Data table
            st.subheader("📋 Detailed Flight Data")
            
            # Sample data every 0.5 seconds for display
            display_df = df[df['time'] % 0.5 < 0.02].copy()
            display_df = display_df.round(2)
            
            st.dataframe(
                display_df[['time', 'altitude', 'velocity', 'thrust', 'phase']],
                use_container_width=True
            )
            
        else:
            st.info("👆 Click 'Run Simulation' to start the rocket trajectory analysis!")
            
            # Show example visualization
            st.subheader("🎯 What You'll Get")
            st.markdown("""
            This simulator provides:
            - **Real-time trajectory calculation** with atmospheric effects
            - **Multi-phase flight modeling** (powered, ballistic, recovery)
            - **Advanced drag calculations** including altitude-dependent air density
            - **Configurable thrust curves** (progressive, neutral, regressive)
            - **Recovery system modeling** with parachute deployment
            - **Detailed performance metrics** and exportable data
            """)

if __name__ == "__main__":
    main()