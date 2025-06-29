// src/RocketSimulator.jsx
import React, { useState, useEffect, useRef } from 'react';
import {
  LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, Legend, ResponsiveContainer,
  ScatterChart, Scatter
} from 'recharts';
import { Play, Pause, RotateCcw, Download, Settings } from 'lucide-react';

const RocketSimulator = () => {
  const [isRunning, setIsRunning] = useState(false);
  const [currentTime, setCurrentTime] = useState(0);
  const [trajectoryData, setTrajectoryData] = useState([]);
  const [rocketPosition, setRocketPosition] = useState({ x: 0, y: 0 });
  const [showSettings, setShowSettings] = useState(false);
  const intervalRef = useRef(null);

  const [params, setParams] = useState({
    totalImpulse: 160,
    burnTime: 2.8,
    propellantMass: 0.24,
    dryMass: 0.8,
    diameter: 0.054,
    length: 0.6,
    dragCoefficient: 0.45,
    launchAngle: 85,
    launchHeight: 0,
    airDensity: 1.225,
    gravity: 9.81,
    windSpeed: 2.0,
    thrustCurveType: 'progressive',
    finEfficiency: 0.85,
    recoveryDeployAltitude: 150,
    recoveryDragCoeff: 1.3,
    recoveryArea: 0.5
  });

  const [simState, setSimState] = useState({
    x: 0, y: 0, vx: 0, vy: 0, ax: 0, ay: 0, mass: 0, thrust: 0, drag: 0,
    phase: 'powered', maxAltitude: 0, maxVelocity: 0, burnoutTime: 0, apogeeTime: 0
  });

  const getThrustAtTime = (t, burnTime, totalImpulse, curveType) => {
    if (t > burnTime) return 0;
    const avgThrust = totalImpulse / burnTime;
    const progress = t / burnTime;
    switch (curveType) {
      case 'progressive': return avgThrust * (0.3 + 1.4 * progress);
      case 'regressive': return avgThrust * (1.7 - 1.4 * progress);
      case 'neutral': return avgThrust;
      default: return avgThrust;
    }
  };

  const getAirDensity = (altitude) => {
    const scaleHeight = 8400;
    return params.airDensity * Math.exp(-altitude / scaleHeight);
  };

  const calculateDrag = (velocity, altitude, area, dragCoeff) => {
    const rho = getAirDensity(altitude);
    const v_mag = Math.sqrt(velocity.x ** 2 + velocity.y ** 2);
    const drag_mag = 0.5 * rho * v_mag ** 2 * area * dragCoeff;
    if (v_mag === 0) return { x: 0, y: 0 };
    return {
      x: -drag_mag * (velocity.x / v_mag),
      y: -drag_mag * (velocity.y / v_mag)
    };
  };

  const simulateStep = (state, dt) => {
    const newState = { ...state };
    const time = currentTime;

    if (time <= params.burnTime) {
      const burnProgress = time / params.burnTime;
      newState.mass = params.dryMass + params.propellantMass * (1 - burnProgress);
    } else {
      newState.mass = params.dryMass;
    }

    newState.thrust = getThrustAtTime(time, params.burnTime, params.totalImpulse, params.thrustCurveType);
    const area = Math.PI * (params.diameter / 2) ** 2;

    if (time <= params.burnTime) newState.phase = 'powered';
    else if (state.vy > 0) newState.phase = 'ballistic';
    else if (state.y > params.recoveryDeployAltitude) newState.phase = 'recovery';
    else newState.phase = 'landed';

    const launchAngleRad = (params.launchAngle * Math.PI) / 180;
    let thrustX = 0, thrustY = 0;
    if (newState.phase === 'powered') {
      thrustX = newState.thrust * Math.cos(launchAngleRad);
      thrustY = newState.thrust * Math.sin(launchAngleRad);
    }

    let dragCoeff = params.dragCoefficient;
    let dragArea = area;
    if (newState.phase === 'recovery') {
      dragCoeff = params.recoveryDragCoeff;
      dragArea = params.recoveryArea;
    }

    const drag = calculateDrag({ x: state.vx, y: state.vy }, state.y, dragArea, dragCoeff);
    const windDrag = -0.1 * (state.vx - params.windSpeed);

    newState.ax = (thrustX + drag.x + windDrag) / newState.mass;
    newState.ay = (thrustY + drag.y - params.gravity * newState.mass) / newState.mass;
    newState.vx = state.vx + newState.ax * dt;
    newState.vy = state.vy + newState.ay * dt;
    newState.x = state.x + newState.vx * dt;
    newState.y = Math.max(0, state.y + newState.vy * dt);

    newState.maxAltitude = Math.max(state.maxAltitude, newState.y);
    const velocity = Math.sqrt(newState.vx ** 2 + newState.vy ** 2);
    newState.maxVelocity = Math.max(state.maxVelocity, velocity);

    if (time > params.burnTime && state.burnoutTime === 0) newState.burnoutTime = time;
    if (newState.vy < 0 && state.vy >= 0 && state.apogeeTime === 0) newState.apogeeTime = time;

    return newState;
  };

  const runSimulation = () => {
    if (isRunning) return;
    setIsRunning(true);
    setCurrentTime(0);
    setTrajectoryData([]);
    let time = 0;
    let state = {
      x: 0, y: params.launchHeight, vx: 0, vy: 0, ax: 0, ay: 0,
      mass: params.dryMass + params.propellantMass, thrust: 0, drag: 0,
      phase: 'powered', maxAltitude: 0, maxVelocity: 0, burnoutTime: 0, apogeeTime: 0
    };
    const trajectory = [];

    intervalRef.current = setInterval(() => {
      for (let i = 0; i < 5; i++) {
        state = simulateStep(state, 0.01);
        time += 0.01;
        if (time % 0.1 < 0.01) {
          trajectory.push({
            time: parseFloat(time.toFixed(2)),
            x: parseFloat(state.x.toFixed(2)),
            y: parseFloat(state.y.toFixed(2)),
            velocity: parseFloat(Math.sqrt(state.vx ** 2 + state.vy ** 2).toFixed(2)),
            altitude: parseFloat(state.y.toFixed(2)),
            thrust: parseFloat(state.thrust.toFixed(2)),
            phase: state.phase
          });
        }
        if (state.y <= 0 && time > 1 || time > 60) {
          setIsRunning(false);
          clearInterval(intervalRef.current);
          break;
        }
      }
      setCurrentTime(time);
      setSimState(state);
      setRocketPosition({ x: state.x, y: state.y });
      setTrajectoryData([...trajectory]);
    }, 50);
  };

  const stopSimulation = () => {
    setIsRunning(false);
    clearInterval(intervalRef.current);
  };

  const resetSimulation = () => {
    stopSimulation();
    setCurrentTime(0);
    setTrajectoryData([]);
    setRocketPosition({ x: 0, y: 0 });
    setSimState({
      x: 0, y: params.launchHeight, vx: 0, vy: 0, ax: 0, ay: 0,
      mass: params.dryMass + params.propellantMass, thrust: 0, drag: 0,
      phase: 'powered', maxAltitude: 0, maxVelocity: 0, burnoutTime: 0, apogeeTime: 0
    });
  };

  const exportData = () => {
    const csv = ['Time,X,Y,Velocity,Altitude,Thrust,Phase']
      .concat(trajectoryData.map(d => `${d.time},${d.x},${d.y},${d.velocity},${d.altitude},${d.thrust},${d.phase}`))
      .join('\n');
    const blob = new Blob([csv], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'rocket_trajectory.csv';
    a.click();
  };

  useEffect(() => () => clearInterval(intervalRef.current), []);

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-blue-900 to-indigo-900 text-white p-6">
      <div className="max-w-7xl mx-auto">
        <div className="text-center mb-8">
          <h1 className="text-4xl font-bold mb-2 bg-gradient-to-r from-blue-400 to-purple-400 bg-clip-text text-transparent">
            Advanced Sugar Rocket Trajectory Simulator
          </h1>
          <p className="text-lg text-blue-200">High-fidelity physics simulation with atmospheric modeling</p>
        </div>
        {/* Add control panel and charts here */}
      </div>
    </div>
  );
};

export default RocketSimulator;
