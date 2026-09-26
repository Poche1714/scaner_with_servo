import React, { useEffect, useRef, useState, useCallback } from 'react';
import * as THREE from 'three';
import {
  BotPose,
  TrajectoryPoint,
  Waypoint,
  RoverSweepState,
  DiscoveredPoint2D,
  MapEnvironmentPreset,
} from '../../types/worldDiscoverer';
import { createRover3DModel, updateRover3D, Rover3DInstance } from '../../utils/rover3DModel';
import { WORLD_PRESETS } from '../../utils/worldSimulator';
import {
  Eye,
  Crosshair,
  Compass,
  Layers,
  Sparkles,
  Maximize2,
  Minimize2,
  Camera,
  Play,
  Pause,
  MapPin,
  RotateCcw,
  Footprints,
  Radio,
  Sun,
  Grid,
  Activity,
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Square,
} from 'lucide-react';

export type CameraViewMode = 'orbit' | 'chase' | 'fpv' | 'top_down';

export interface WorldSimulator3DProps {
  botPose: BotPose;
  trajectory: TrajectoryPoint[];
  waypoints: Waypoint[];
  roverState: RoverSweepState;
  points: DiscoveredPoint2D[];
  preset: MapEnvironmentPreset;
  isAutonomous: boolean;
  onToggleAutonomous: () => void;
  onDriveForward: () => void;
  onDriveBackward: () => void;
  onTurnLeft: () => void;
  onTurnRight: () => void;
  onStopBot: () => void;
  onAddWaypoint: () => void;
  onResetRoute: () => void;
}

export const WorldSimulator3D: React.FC<WorldSimulator3DProps> = ({
  botPose,
  trajectory,
  waypoints,
  roverState,
  points,
  preset,
  isAutonomous,
  onToggleAutonomous,
  onDriveForward,
  onDriveBackward,
  onTurnLeft,
  onTurnRight,
  onStopBot,
  onAddWaypoint,
  onResetRoute,
}) => {
  const mountRef = useRef<HTMLDivElement>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  // Viewport Settings
  const [cameraMode, setCameraMode] = useState<CameraViewMode>('orbit');
  const [showGrid, setShowGrid] = useState(true);
  const [showBeam, setShowBeam] = useState(true);
  const [showPointcloud, setShowPointcloud] = useState(true);
  const [showShadows, setShowShadows] = useState(true);
  const [isFullscreen, setIsFullscreen] = useState(false);

  // References for Three.js engine
  const threeRef = useRef<{
    renderer: THREE.WebGLRenderer;
    scene: THREE.Scene;
    camera: THREE.PerspectiveCamera;
    roverInstance: Rover3DInstance;
    trajectoryLine: THREE.Line;
    trajectoryPositions: Float32Array;
    waypointsGroup: THREE.Group;
    obstaclesGroup: THREE.Group;
    pointcloudPoints: THREE.Points;
    pointcloudGeo: THREE.BufferGeometry;
    gridHelper: THREE.GridHelper;
    groundMesh: THREE.Mesh;
    dirLight: THREE.DirectionalLight;
    controls: {
      isDragging: boolean;
      dragButton: number;
      prevMouse: { x: number; y: number };
      spherical: { radius: number; theta: number; phi: number };
      target: THREE.Vector3;
    };
    animFrameId: number;
    lastTime: number;
  } | null>(null);

  // Track latest props in refs to avoid re-instantiating Three.js scene
  const botPoseRef = useRef(botPose);
  botPoseRef.current = botPose;

  const roverStateRef = useRef(roverState);
  roverStateRef.current = roverState;

  const trajectoryRef = useRef(trajectory);
  trajectoryRef.current = trajectory;

  const cameraModeRef = useRef(cameraMode);
  cameraModeRef.current = cameraMode;

  const waypointsRef = useRef(waypoints);
  waypointsRef.current = waypoints;

  const pointsRef = useRef(points);
  pointsRef.current = points;

  // Initialize Three.js Simulation
  useEffect(() => {
    const container = mountRef.current;
    const canvas = canvasRef.current;
    if (!container || !canvas) return;

    const width = container.clientWidth || 800;
    const height = container.clientHeight || 560;

    // 1. WebGL Renderer
    const renderer = new THREE.WebGLRenderer({
      canvas,
      antialias: true,
      powerPreference: 'high-performance',
      alpha: false,
      preserveDrawingBuffer: true,
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;

    // 2. Scene with Gazebo simulation atmosphere
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x181c22); // Gazebo slate dark grey sky
    scene.fog = new THREE.FogExp2(0x181c22, 0.035);

    // 3. Camera
    const camera = new THREE.PerspectiveCamera(50, width / height, 0.05, 500);
    camera.position.set(0, 2.5, 3.8);

    // Orbit Camera State
    const controls = {
      isDragging: false,
      dragButton: 0, // 0 = left (rotate), 2 = right (pan)
      prevMouse: { x: 0, y: 0 },
      spherical: { radius: 4.5, theta: Math.PI / 4, phi: Math.PI / 3.2 },
      target: new THREE.Vector3(0, 0.2, 0),
    };

    // 4. Lighting (Simulating Sun & Studio Lighting in Gazebo)
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xfffbeb, 1.4);
    dirLight.position.set(6, 12, 8);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.camera.near = 0.5;
    dirLight.shadow.camera.far = 40;
    dirLight.shadow.camera.left = -10;
    dirLight.shadow.camera.right = 10;
    dirLight.shadow.camera.top = 10;
    dirLight.shadow.camera.bottom = -10;
    dirLight.shadow.bias = -0.0005;
    scene.add(dirLight);

    // Subtle blue fill light from opposite angle
    const fillLight = new THREE.DirectionalLight(0x38bdf8, 0.35);
    fillLight.position.set(-6, 4, -8);
    scene.add(fillLight);

    // 5. Gazebo Ground Plane with Texture Grid
    const groundGeo = new THREE.PlaneGeometry(60, 60);
    const groundMat = new THREE.MeshStandardMaterial({
      color: 0x22272e, // Gazebo concrete asphalt ground
      roughness: 0.85,
      metalness: 0.1,
    });
    const groundMesh = new THREE.Mesh(groundGeo, groundMat);
    groundMesh.rotation.x = -Math.PI / 2;
    groundMesh.receiveShadow = true;
    scene.add(groundMesh);

    // High contrast 1m and 0.2m Gazebo Grid
    const gridHelper = new THREE.GridHelper(60, 60, 0x06b6d4, 0x334155);
    gridHelper.position.y = 0.002; // Slightly above ground
    scene.add(gridHelper);

    // 6. Base / Start Station Landing Pad
    const baseGroup = new THREE.Group();
    baseGroup.position.set(0, 0.005, 0);

    const basePadMesh = new THREE.Mesh(
      new THREE.RingGeometry(0.35, 0.42, 32),
      new THREE.MeshBasicMaterial({ color: 0x10b981, side: THREE.DoubleSide })
    );
    basePadMesh.rotation.x = -Math.PI / 2;
    baseGroup.add(basePadMesh);

    const baseInnerMesh = new THREE.Mesh(
      new THREE.CircleGeometry(0.35, 32),
      new THREE.MeshBasicMaterial({ color: 0x064e3b, transparent: true, opacity: 0.5, side: THREE.DoubleSide })
    );
    baseInnerMesh.rotation.x = -Math.PI / 2;
    baseGroup.add(baseInnerMesh);

    scene.add(baseGroup);

    // 7. 3D Rover Instance
    const roverInstance = createRover3DModel();
    scene.add(roverInstance.rootGroup);

    // 8. 3D Trajectory Ribbon (Historical Path Line on Ground)
    const MAX_TRAJECTORY_POINTS = 3000;
    const trajectoryPositions = new Float32Array(MAX_TRAJECTORY_POINTS * 3);
    const trajectoryGeo = new THREE.BufferGeometry();
    trajectoryGeo.setAttribute('position', new THREE.BufferAttribute(trajectoryPositions, 3));
    trajectoryGeo.setDrawRange(0, 0);

    const trajectoryMat = new THREE.LineBasicMaterial({
      color: 0x06b6d4, // Cyan glowing route line
      linewidth: 3,
    });
    const trajectoryLine = new THREE.Line(trajectoryGeo, trajectoryMat);
    scene.add(trajectoryLine);

    // 9. Waypoints Group in 3D
    const waypointsGroup = new THREE.Group();
    scene.add(waypointsGroup);

    // 10. Obstacles Group (3D Crate cubes, pillars, and walls matching Gazebo reference)
    const obstaclesGroup = new THREE.Group();
    scene.add(obstaclesGroup);

    // 11. Sonar Pointcloud in 3D (Detected Obstacle Hits)
    const MAX_PCD_POINTS = 2000;
    const pcdPositions = new Float32Array(MAX_PCD_POINTS * 3);
    const pcdColors = new Float32Array(MAX_PCD_POINTS * 3);
    const pointcloudGeo = new THREE.BufferGeometry();
    pointcloudGeo.setAttribute('position', new THREE.BufferAttribute(pcdPositions, 3));
    pointcloudGeo.setAttribute('color', new THREE.BufferAttribute(pcdColors, 3));
    pointcloudGeo.setDrawRange(0, 0);

    const pointcloudMat = new THREE.PointsMaterial({
      size: 0.08,
      vertexColors: true,
      transparent: true,
      opacity: 0.9,
    });
    const pointcloudPoints = new THREE.Points(pointcloudGeo, pointcloudMat);
    scene.add(pointcloudPoints);

    // Store references
    threeRef.current = {
      renderer,
      scene,
      camera,
      roverInstance,
      trajectoryLine,
      trajectoryPositions,
      waypointsGroup,
      obstaclesGroup,
      pointcloudPoints,
      pointcloudGeo,
      gridHelper,
      groundMesh,
      dirLight,
      controls,
      animFrameId: 0,
      lastTime: performance.now(),
    };

    // Build the 3D obstacles for the active preset
    rebuildPresetObstacles(obstaclesGroup, preset);

    // Animation Loop
    const animate = (time: number) => {
      const state = threeRef.current;
      if (!state) return;

      const deltaSec = Math.min((time - state.lastTime) / 1000, 0.1);
      state.lastTime = time;

      const currentBot = botPoseRef.current;
      const currentRoverState = roverStateRef.current;
      const currentCamMode = cameraModeRef.current;

      // Update Rover transformations, wheels spinning, sonar servo rotation
      updateRover3D(
        state.roverInstance,
        currentBot.x,
        currentBot.y,
        currentBot.heading,
        currentRoverState.currentAngle,
        currentRoverState.currentDistanceCm,
        currentRoverState.maxRangeCm,
        currentBot.speed,
        currentBot.isMoving,
        deltaSec
      );

      // Pulse wave in sonar beam
      const pulseZ = ((time * 0.002) % 1) * - (Math.min(currentRoverState.currentDistanceCm, currentRoverState.maxRangeCm) / 100);
      state.roverInstance.beamPulse.position.z = pulseZ;

      // Update Camera based on active mode
      const roverPos = state.roverInstance.rootGroup.position;
      const roverHeadingRad = ((currentBot.heading - 90) * Math.PI) / 180;

      if (currentCamMode === 'chase') {
        // Chase Cam: smooth follow from behind the rover
        const chaseDist = 2.4;
        const chaseHeight = 1.35;
        // Direction behind robot is +Z in robot local frame
        const forwardX = Math.cos(-roverHeadingRad - Math.PI / 2);
        const forwardZ = Math.sin(-roverHeadingRad - Math.PI / 2);

        const targetCamX = roverPos.x - forwardX * chaseDist;
        const targetCamZ = roverPos.z - forwardZ * chaseDist;
        const targetCamY = roverPos.y + chaseHeight;

        state.camera.position.lerp(new THREE.Vector3(targetCamX, targetCamY, targetCamZ), 0.15);
        state.camera.lookAt(roverPos.x, roverPos.y + 0.3, roverPos.z);
      } else if (currentCamMode === 'fpv') {
        // FPV: Onboard camera right above ultrasonic sensor
        const forwardX = Math.cos(-roverHeadingRad - Math.PI / 2);
        const forwardZ = Math.sin(-roverHeadingRad - Math.PI / 2);

        state.camera.position.set(roverPos.x + forwardX * 0.08, roverPos.y + 0.28, roverPos.z + forwardZ * 0.08);
        const lookTarget = new THREE.Vector3(
          roverPos.x + forwardX * 3.5,
          roverPos.y + 0.22,
          roverPos.z + forwardZ * 3.5
        );
        state.camera.lookAt(lookTarget);
      } else if (currentCamMode === 'top_down') {
        // Top-Down Ortho-like perspective looking straight down
        state.camera.position.set(roverPos.x, 8.5, roverPos.z + 0.01);
        state.camera.lookAt(roverPos.x, 0, roverPos.z);
      } else {
        // Orbit Camera mode: smooth tracking of orbit target
        const sph = state.controls.spherical;
        const target = state.controls.target;

        // Smoothly bring target near rover if user is driving
        if (currentBot.isMoving) {
          target.lerp(new THREE.Vector3(roverPos.x, 0.2, roverPos.z), 0.08);
        }

        const camX = target.x + sph.radius * Math.sin(sph.phi) * Math.sin(sph.theta);
        const camY = target.y + sph.radius * Math.cos(sph.phi);
        const camZ = target.z + sph.radius * Math.sin(sph.phi) * Math.cos(sph.theta);

        state.camera.position.set(camX, camY, camZ);
        state.camera.lookAt(target);
      }

      state.renderer.render(state.scene, state.camera);
      state.animFrameId = requestAnimationFrame(animate);
    };

    threeRef.current.animFrameId = requestAnimationFrame(animate);

    // Resize Observer
    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const w = entry.contentRect.width;
        const h = entry.contentRect.height;
        if (w > 0 && h > 0 && threeRef.current) {
          threeRef.current.renderer.setSize(w, h);
          threeRef.current.camera.aspect = w / h;
          threeRef.current.camera.updateProjectionMatrix();
        }
      }
    });
    resizeObserver.observe(container);

    return () => {
      resizeObserver.disconnect();
      if (threeRef.current) {
        cancelAnimationFrame(threeRef.current.animFrameId);
        threeRef.current.renderer.dispose();
      }
    };
  }, []);

  // Update Obstacles whenever preset changes
  useEffect(() => {
    if (!threeRef.current) return;
    rebuildPresetObstacles(threeRef.current.obstaclesGroup, preset);
  }, [preset]);

  // Update Trajectory Buffer in 3D
  useEffect(() => {
    const state = threeRef.current;
    if (!state) return;

    const count = Math.min(trajectory.length, 3000);
    const positions = state.trajectoryPositions;

    for (let i = 0; i < count; i++) {
      const p = trajectory[i];
      // Convert cm to meters: x cm -> X m, y cm -> -Z m
      positions[i * 3 + 0] = p.x / 100;
      positions[i * 3 + 1] = 0.035; // slightly above ground to prevent z-fighting
      positions[i * 3 + 2] = -p.y / 100;
    }

    state.trajectoryLine.geometry.attributes.position.needsUpdate = true;
    state.trajectoryLine.geometry.setDrawRange(0, count);
  }, [trajectory]);

  // Update Waypoint Markers in 3D
  useEffect(() => {
    const state = threeRef.current;
    if (!state) return;

    const group = state.waypointsGroup;
    // Clear old waypoints
    while (group.children.length > 0) {
      const child = group.children[0];
      group.remove(child);
    }

    waypoints.forEach((wp, index) => {
      const wpGroup = new THREE.Group();
      wpGroup.position.set(wp.x / 100, 0, -wp.y / 100);

      // Vertical holographic light beacon pole
      const poleGeo = new THREE.CylinderGeometry(0.015, 0.015, 0.8, 8);
      const poleMat = new THREE.MeshBasicMaterial({
        color: index === 0 ? 0x10b981 : 0x06b6d4,
        transparent: true,
        opacity: 0.7,
      });
      const pole = new THREE.Mesh(poleGeo, poleMat);
      pole.position.y = 0.4;
      wpGroup.add(pole);

      // Glowing floating beacon sphere
      const sphereGeo = new THREE.SphereGeometry(0.06, 12, 12);
      const sphereMat = new THREE.MeshStandardMaterial({
        color: index === 0 ? 0x10b981 : 0x06b6d4,
        emissive: index === 0 ? 0x059669 : 0x0891b2,
        emissiveIntensity: 0.9,
      });
      const sphere = new THREE.Mesh(sphereGeo, sphereMat);
      sphere.position.y = 0.8;
      wpGroup.add(sphere);

      // Base footprint ring
      const ringGeo = new THREE.RingGeometry(0.12, 0.16, 20);
      const ringMat = new THREE.MeshBasicMaterial({
        color: index === 0 ? 0x10b981 : 0x06b6d4,
        side: THREE.DoubleSide,
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.rotation.x = -Math.PI / 2;
      ring.position.y = 0.005;
      wpGroup.add(ring);

      group.add(wpGroup);
    });
  }, [waypoints]);

  // Update Sonar Hit Points in 3D
  useEffect(() => {
    const state = threeRef.current;
    if (!state) return;

    const geo = state.pointcloudGeo;
    const posAttr = geo.attributes.position as THREE.BufferAttribute;
    const colAttr = geo.attributes.color as THREE.BufferAttribute;

    const count = Math.min(points.length, 2000);
    const positions = posAttr.array as Float32Array;
    const colors = colAttr.array as Float32Array;

    for (let i = 0; i < count; i++) {
      const p = points[i];
      // World coordinates in meters
      positions[i * 3 + 0] = p.worldX / 100;
      positions[i * 3 + 1] = 0.18; // Sensor height
      positions[i * 3 + 2] = -p.worldY / 100;

      // Color based on type / distance
      if (p.type === 'anomaly') {
        colors[i * 3 + 0] = 0.93; // Purple / Pink
        colors[i * 3 + 1] = 0.28;
        colors[i * 3 + 2] = 0.60;
      } else if (p.distanceCm < 60) {
        colors[i * 3 + 0] = 0.96; // Rose / Red
        colors[i * 3 + 1] = 0.25;
        colors[i * 3 + 2] = 0.37;
      } else {
        colors[i * 3 + 0] = 0.02; // Cyan / Emerald
        colors[i * 3 + 1] = 0.71;
        colors[i * 3 + 2] = 0.83;
      }
    }

    posAttr.needsUpdate = true;
    colAttr.needsUpdate = true;
    geo.setDrawRange(0, count);
  }, [points]);

  // Toggles update
  useEffect(() => {
    if (!threeRef.current) return;
    threeRef.current.gridHelper.visible = showGrid;
    threeRef.current.roverInstance.beamGroup.visible = showBeam;
    threeRef.current.pointcloudPoints.visible = showPointcloud;
    threeRef.current.dirLight.castShadow = showShadows;
  }, [showGrid, showBeam, showPointcloud, showShadows]);

  // Mouse / Touch Event Handlers for Gazebo Orbit Camera
  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    if (cameraModeRef.current !== 'orbit') return;
    const state = threeRef.current;
    if (!state) return;

    state.controls.isDragging = true;
    state.controls.dragButton = e.button;
    state.controls.prevMouse = { x: e.clientX, y: e.clientY };
  }, []);

  const handleMouseMove = useCallback((e: React.MouseEvent) => {
    const state = threeRef.current;
    if (!state || !state.controls.isDragging || cameraModeRef.current !== 'orbit') return;

    const dx = e.clientX - state.controls.prevMouse.x;
    const dy = e.clientY - state.controls.prevMouse.y;
    state.controls.prevMouse = { x: e.clientX, y: e.clientY };

    if (state.controls.dragButton === 0) {
      // Left click: Orbit Rotation
      state.controls.spherical.theta -= dx * 0.007;
      state.controls.spherical.phi = Math.max(
        0.1,
        Math.min(Math.PI / 2 - 0.05, state.controls.spherical.phi - dy * 0.007)
      );
    } else if (state.controls.dragButton === 2) {
      // Right click: Pan
      const panSpeed = 0.004 * (state.controls.spherical.radius / 5);
      const theta = state.controls.spherical.theta;
      state.controls.target.x -= (dx * Math.cos(theta) - dy * Math.sin(theta)) * panSpeed;
      state.controls.target.z += (dx * Math.sin(theta) + dy * Math.cos(theta)) * panSpeed;
    }
  }, []);

  const handleMouseUp = useCallback(() => {
    if (!threeRef.current) return;
    threeRef.current.controls.isDragging = false;
  }, []);

  const handleWheel = useCallback((e: React.WheelEvent) => {
    if (cameraModeRef.current !== 'orbit') return;
    const state = threeRef.current;
    if (!state) return;

    e.preventDefault();
    const zoomFactor = e.deltaY * 0.003;
    state.controls.spherical.radius = Math.max(
      1.2,
      Math.min(18.0, state.controls.spherical.radius + zoomFactor)
    );
  }, []);

  // Context menu prevention for right-click drag pan
  const handleContextMenu = useCallback((e: React.MouseEvent) => {
    e.preventDefault();
  }, []);

  // Focus camera back on the bot
  const handleFocusBot = useCallback(() => {
    if (!threeRef.current) return;
    const pos = threeRef.current.roverInstance.rootGroup.position;
    threeRef.current.controls.target.set(pos.x, 0.2, pos.z);
    threeRef.current.controls.spherical.radius = 4.0;
    threeRef.current.controls.spherical.phi = Math.PI / 3.4;
  }, []);

  // Take Snapshot of 3D Simulation
  const handleTakeSnapshot = useCallback(() => {
    if (!canvasRef.current) return;
    const dataUrl = canvasRef.current.toDataURL('image/png');
    const link = document.createElement('a');
    link.download = `gazebo-rover-sim-${Date.now()}.png`;
    link.href = dataUrl;
    link.click();
  }, []);

  return (
    <div
      ref={mountRef}
      className={`relative w-full rounded-xl overflow-hidden border border-neutral-800 bg-neutral-950 select-none shadow-2xl transition-all ${
        isFullscreen ? 'fixed inset-0 z-50 rounded-none h-screen' : 'h-[520px] md:h-[600px]'
      }`}
      onMouseDown={handleMouseDown}
      onMouseMove={handleMouseMove}
      onMouseUp={handleMouseUp}
      onWheel={handleWheel}
      onContextMenu={handleContextMenu}
    >
      {/* 3D WebGL Canvas */}
      <canvas ref={canvasRef} className="w-full h-full block cursor-grab active:cursor-grabbing" />

      {/* Top Left: Gazebo ROS Simulation Badge & Preset Info */}
      <div className="absolute top-3 left-3 flex flex-col gap-1.5 pointer-events-none z-10">
        <div className="flex items-center gap-2 bg-neutral-900/90 backdrop-blur-md px-3 py-1.5 rounded-lg border border-neutral-700/80 shadow-lg pointer-events-auto">
          <div className="w-2.5 h-2.5 rounded-full bg-amber-400 animate-pulse" />
          <span className="text-xs font-bold tracking-wide text-neutral-100 flex items-center gap-1.5 font-mono">
            <span>GAZEBO 3D SIMULATOR</span>
            <span className="text-amber-400 text-[10px] bg-amber-500/20 px-1.5 py-0.5 rounded border border-amber-500/30">
              ROVER 4x4
            </span>
          </span>
          <span className="text-neutral-500 text-xs">|</span>
          <span className="text-[11px] text-neutral-300 font-medium">
            {WORLD_PRESETS[preset]?.name || 'Escenario 3D'}
          </span>
        </div>

        {/* Real-time Telemetry Overlay */}
        <div className="flex flex-wrap items-center gap-2 text-[11px] font-mono text-neutral-300">
          <div className="bg-neutral-900/85 backdrop-blur-md px-2.5 py-1 rounded-md border border-neutral-800 flex items-center gap-1.5">
            <Radio className="w-3.5 h-3.5 text-cyan-400" />
            <span>SONAR:</span>
            <span
              className={`font-bold ${
                roverState.currentDistanceCm < 50
                  ? 'text-rose-400'
                  : roverState.currentDistanceCm < 100
                  ? 'text-amber-400'
                  : 'text-emerald-400'
              }`}
            >
              {Math.round(roverState.currentDistanceCm)} cm
            </span>
            <span className="text-neutral-500">(@ {Math.round(roverState.currentAngle)}°)</span>
          </div>

          <div className="bg-neutral-900/85 backdrop-blur-md px-2.5 py-1 rounded-md border border-neutral-800 flex items-center gap-1.5">
            <Footprints className="w-3.5 h-3.5 text-amber-400" />
            <span>RUTA:</span>
            <span className="font-bold text-neutral-100">
              {(botPose.totalDistanceCm / 100).toFixed(2)} m
            </span>
          </div>

          <div className="bg-neutral-900/85 backdrop-blur-md px-2.5 py-1 rounded-md border border-neutral-800 flex items-center gap-1.5">
            <Compass className="w-3.5 h-3.5 text-purple-400" />
            <span>POS:</span>
            <span className="text-neutral-200">
              [{Math.round(botPose.x)}, {Math.round(botPose.y)}]cm
            </span>
          </div>
        </div>
      </div>

      {/* Top Right: Camera Mode Switcher & Viewport Actions */}
      <div className="absolute top-3 right-3 flex items-center gap-1.5 z-10">
        {/* Camera Selector */}
        <div className="bg-neutral-900/90 backdrop-blur-md p-1 rounded-lg border border-neutral-800 flex items-center gap-1 shadow-lg">
          <button
            onClick={() => setCameraMode('orbit')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-colors cursor-pointer ${
              cameraMode === 'orbit'
                ? 'bg-amber-500 text-black font-semibold'
                : 'text-neutral-300 hover:text-neutral-100 hover:bg-neutral-800'
            }`}
            title="Cámara Orbital Libre (Arrastrar para girar, clic derecho para paneo)"
          >
            Orbital
          </button>
          <button
            onClick={() => setCameraMode('chase')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-colors cursor-pointer ${
              cameraMode === 'chase'
                ? 'bg-amber-500 text-black font-semibold'
                : 'text-neutral-300 hover:text-neutral-100 hover:bg-neutral-800'
            }`}
            title="Cámara Persecución (Sigue al robot desde atrás)"
          >
            Seguir Bot
          </button>
          <button
            onClick={() => setCameraMode('fpv')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-colors cursor-pointer ${
              cameraMode === 'fpv'
                ? 'bg-amber-500 text-black font-semibold'
                : 'text-neutral-300 hover:text-neutral-100 hover:bg-neutral-800'
            }`}
            title="Cámara a Bordo (Primera Persona / FPV sobre el sensor)"
          >
            FPV
          </button>
          <button
            onClick={() => setCameraMode('top_down')}
            className={`px-2.5 py-1 text-xs rounded font-medium transition-colors cursor-pointer ${
              cameraMode === 'top_down'
                ? 'bg-amber-500 text-black font-semibold'
                : 'text-neutral-300 hover:text-neutral-100 hover:bg-neutral-800'
            }`}
            title="Vista Cenital Superior (Plano táctico 3D)"
          >
            Cenital
          </button>
        </div>

        {/* Focus Bot Button */}
        <button
          onClick={handleFocusBot}
          className="p-2 bg-neutral-900/90 hover:bg-neutral-800 text-neutral-300 hover:text-neutral-100 rounded-lg border border-neutral-800 backdrop-blur-md shadow-lg transition-colors cursor-pointer"
          title="Centrar Cámara en el Robot"
        >
          <Crosshair className="w-4 h-4 text-cyan-400" />
        </button>

        {/* Snapshot / Camera capture */}
        <button
          onClick={handleTakeSnapshot}
          className="p-2 bg-neutral-900/90 hover:bg-neutral-800 text-neutral-300 hover:text-neutral-100 rounded-lg border border-neutral-800 backdrop-blur-md shadow-lg transition-colors cursor-pointer"
          title="Capturar Imagen 3D"
        >
          <Camera className="w-4 h-4 text-emerald-400" />
        </button>

        {/* Fullscreen Toggle */}
        <button
          onClick={() => setIsFullscreen(!isFullscreen)}
          className="p-2 bg-neutral-900/90 hover:bg-neutral-800 text-neutral-300 hover:text-neutral-100 rounded-lg border border-neutral-800 backdrop-blur-md shadow-lg transition-colors cursor-pointer"
          title={isFullscreen ? 'Salir de Pantalla Completa' : 'Pantalla Completa'}
        >
          {isFullscreen ? <Minimize2 className="w-4 h-4" /> : <Maximize2 className="w-4 h-4" />}
        </button>
      </div>

      {/* Bottom Left: Visual Layer Toggles (Grid, Beam, PCD, Shadows) */}
      <div className="absolute bottom-3 left-3 flex items-center gap-1.5 z-10">
        <div className="bg-neutral-900/85 backdrop-blur-md p-1 rounded-lg border border-neutral-800 flex items-center gap-1 shadow-lg text-[11px]">
          <button
            onClick={() => setShowGrid(!showGrid)}
            className={`px-2 py-1 rounded flex items-center gap-1 transition-colors cursor-pointer ${
              showGrid ? 'bg-neutral-800 text-cyan-300 font-medium' : 'text-neutral-500 hover:text-neutral-300'
            }`}
            title="Alternar Cuadrícula Gazebo 3D"
          >
            <Grid className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Cuadrícula</span>
          </button>

          <button
            onClick={() => setShowBeam(!showBeam)}
            className={`px-2 py-1 rounded flex items-center gap-1 transition-colors cursor-pointer ${
              showBeam ? 'bg-neutral-800 text-cyan-300 font-medium' : 'text-neutral-500 hover:text-neutral-300'
            }`}
            title="Alternar Haz Ultrasónico 3D"
          >
            <Radio className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Haz Sonar</span>
          </button>

          <button
            onClick={() => setShowPointcloud(!showPointcloud)}
            className={`px-2 py-1 rounded flex items-center gap-1 transition-colors cursor-pointer ${
              showPointcloud ? 'bg-neutral-800 text-purple-300 font-medium' : 'text-neutral-500 hover:text-neutral-300'
            }`}
            title="Alternar Nube de Puntos de Obstáculos 3D"
          >
            <Layers className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Obstáculos 3D</span>
          </button>

          <button
            onClick={() => setShowShadows(!showShadows)}
            className={`px-2 py-1 rounded flex items-center gap-1 transition-colors cursor-pointer ${
              showShadows ? 'bg-neutral-800 text-amber-300 font-medium' : 'text-neutral-500 hover:text-neutral-300'
            }`}
            title="Alternar Sombras en Tiempo Real"
          >
            <Sun className="w-3.5 h-3.5" />
            <span className="hidden sm:inline">Sombras</span>
          </button>
        </div>
      </div>

      {/* Bottom Right: Quick Drive D-Pad & Waypoint Floating Controls */}
      <div className="absolute bottom-3 right-3 flex flex-col items-end gap-2 z-10">
        {/* On-screen Drive D-Pad overlay for quick tactile driving */}
        <div className="bg-neutral-900/90 backdrop-blur-md p-2 rounded-xl border border-neutral-800 shadow-xl flex flex-col items-center gap-1">
          {/* Forward */}
          <button
            onClick={onDriveForward}
            disabled={isAutonomous}
            className="w-9 h-8 bg-neutral-800 hover:bg-neutral-700 active:bg-amber-500 active:text-black text-neutral-200 rounded-md flex items-center justify-center transition-colors cursor-pointer disabled:opacity-40"
            title="Avanzar (W)"
          >
            <ArrowUp className="w-4 h-4" />
          </button>

          {/* Left - Stop - Right */}
          <div className="flex items-center gap-1">
            <button
              onClick={onTurnLeft}
              disabled={isAutonomous}
              className="w-9 h-8 bg-neutral-800 hover:bg-neutral-700 active:bg-amber-500 active:text-black text-neutral-200 rounded-md flex items-center justify-center transition-colors cursor-pointer disabled:opacity-40"
              title="Girar Izquierda (A)"
            >
              <ArrowLeft className="w-4 h-4" />
            </button>

            <button
              onClick={onStopBot}
              className="w-9 h-8 bg-rose-500/20 hover:bg-rose-500/30 active:bg-rose-600 text-rose-400 active:text-white rounded-md flex items-center justify-center transition-colors cursor-pointer"
              title="Detener (Espacio)"
            >
              <Square className="w-3.5 h-3.5 fill-current" />
            </button>

            <button
              onClick={onTurnRight}
              disabled={isAutonomous}
              className="w-9 h-8 bg-neutral-800 hover:bg-neutral-700 active:bg-amber-500 active:text-black text-neutral-200 rounded-md flex items-center justify-center transition-colors cursor-pointer disabled:opacity-40"
              title="Girar Derecha (D)"
            >
              <ArrowRight className="w-4 h-4" />
            </button>
          </div>

          {/* Backward */}
          <button
            onClick={onDriveBackward}
            disabled={isAutonomous}
            className="w-9 h-8 bg-neutral-800 hover:bg-neutral-700 active:bg-amber-500 active:text-black text-neutral-200 rounded-md flex items-center justify-center transition-colors cursor-pointer disabled:opacity-40"
            title="Retroceder (S)"
          >
            <ArrowDown className="w-4 h-4" />
          </button>
        </div>

        {/* Quick Route Buttons */}
        <div className="flex items-center gap-1 bg-neutral-900/90 backdrop-blur-md p-1 rounded-lg border border-neutral-800 shadow-lg text-xs">
          <button
            onClick={onToggleAutonomous}
            className={`px-2.5 py-1.5 rounded font-medium flex items-center gap-1 transition-colors cursor-pointer ${
              isAutonomous
                ? 'bg-purple-600 hover:bg-purple-500 text-white animate-pulse'
                : 'bg-neutral-800 hover:bg-neutral-700 text-neutral-200'
            }`}
            title="Alternar Exploración y Mapeo Autónomo"
          >
            <Sparkles className="w-3.5 h-3.5 text-purple-300" />
            <span>{isAutonomous ? 'Auto: ON' : 'Auto-Mapeo'}</span>
          </button>

          <button
            onClick={onAddWaypoint}
            className="px-2.5 py-1.5 bg-cyan-950/60 hover:bg-cyan-900/80 text-cyan-300 border border-cyan-500/30 rounded font-medium flex items-center gap-1 transition-colors cursor-pointer"
            title="Fijar Hito en la Ruta Actual"
          >
            <MapPin className="w-3.5 h-3.5" />
            <span>Hito #{waypoints.length + 1}</span>
          </button>

          <button
            onClick={onResetRoute}
            className="p-1.5 bg-neutral-800 hover:bg-neutral-700 text-neutral-400 hover:text-neutral-100 rounded transition-colors cursor-pointer"
            title="Reiniciar Posición y Ruta del Robot"
          >
            <RotateCcw className="w-3.5 h-3.5" />
          </button>
        </div>
      </div>
    </div>
  );
};

/**
 * Helper to build 3D obstacle meshes (crates/cubes like Gazebo screenshot, pillars, walls)
 * based on the active environment preset.
 */
function rebuildPresetObstacles(group: THREE.Group, preset: MapEnvironmentPreset) {
  // Clear old meshes
  while (group.children.length > 0) {
    const child = group.children[0];
    group.remove(child);
  }

  const presetData = WORLD_PRESETS[preset];
  if (!presetData) return;

  // Materials
  // 1. Gazebo Crate / Box Material (just like the crate in the user's reference image)
  const crateMat = new THREE.MeshStandardMaterial({
    color: 0x854d0e, // Industrial Timber Crate / Gazebo Brown Box
    roughness: 0.65,
    metalness: 0.1,
  });

  // 2. Concrete Pillar Material
  const pillarMat = new THREE.MeshStandardMaterial({
    color: 0x475569, // Slate Concrete
    roughness: 0.8,
    metalness: 0.2,
  });

  // 3. Wall Material
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0x334155, // Dark slate laboratory wall
    roughness: 0.7,
    metalness: 0.15,
  });

  // 4. Anomaly / Relic Material
  const anomalyMat = new THREE.MeshStandardMaterial({
    color: 0x8b5cf6,
    emissive: 0x6d28d9,
    emissiveIntensity: 0.5,
    roughness: 0.3,
    metalness: 0.6,
  });

  // Build Walls as 3D extruded boxes
  presetData.walls.forEach((w) => {
    // Map coordinates from cm to meters
    const x1 = w.x1 / 100;
    const z1 = -w.y1 / 100;
    const x2 = w.x2 / 100;
    const z2 = -w.y2 / 100;

    const dx = x2 - x1;
    const dz = z2 - z1;
    const length = Math.hypot(dx, dz);
    const angle = Math.atan2(dz, dx);

    const wallHeight = w.type === 'obstacle' ? 0.6 : 0.8;
    const wallThickness = 0.12;

    const wallGeo = new THREE.BoxGeometry(length, wallHeight, wallThickness);
    const wallMesh = new THREE.Mesh(wallGeo, w.type === 'obstacle' ? crateMat : wallMat);

    // Center position
    wallMesh.position.set((x1 + x2) / 2, wallHeight / 2, (z1 + z2) / 2);
    wallMesh.rotation.y = -angle;
    wallMesh.castShadow = true;
    wallMesh.receiveShadow = true;

    group.add(wallMesh);
  });

  // Build Cylinders / Crates from CircularObstacles
  presetData.circles.forEach((c) => {
    const cx = c.cx / 100;
    const cz = -c.cy / 100;
    const radius = c.radius / 100;

    if (c.type === 'anomaly') {
      // Ancient Relic Monolith
      const monolithGeo = new THREE.OctahedronGeometry(radius * 1.2, 0);
      const monolithMesh = new THREE.Mesh(monolithGeo, anomalyMat);
      monolithMesh.position.set(cx, radius * 1.4, cz);
      monolithMesh.castShadow = true;
      monolithMesh.receiveShadow = true;
      group.add(monolithMesh);
    } else if (c.radius > 18) {
      // Large obstacle: Render as a Gazebo Cargo Crate / Cube (like in reference image!)
      const boxSize = radius * 1.8;
      const boxGeo = new THREE.BoxGeometry(boxSize, boxSize, boxSize);
      const boxMesh = new THREE.Mesh(boxGeo, crateMat);
      boxMesh.position.set(cx, boxSize / 2, cz);
      boxMesh.castShadow = true;
      boxMesh.receiveShadow = true;

      // Add metal corner brackets to look exactly like the Gazebo crate
      const frameGeo = new THREE.BoxGeometry(boxSize * 1.02, boxSize * 0.08, boxSize * 1.02);
      const frameMat = new THREE.MeshStandardMaterial({ color: 0x1f2937, metalness: 0.8, roughness: 0.3 });
      const frameMesh = new THREE.Mesh(frameGeo, frameMat);
      frameMesh.position.set(cx, boxSize / 2, cz);
      group.add(frameMesh);

      group.add(boxMesh);
    } else {
      // Cylinder Pillar
      const height = 0.75;
      const cylGeo = new THREE.CylinderGeometry(radius, radius, height, 20);
      const cylMesh = new THREE.Mesh(cylGeo, pillarMat);
      cylMesh.position.set(cx, height / 2, cz);
      cylMesh.castShadow = true;
      cylMesh.receiveShadow = true;
      group.add(cylMesh);
    }
  });
}
