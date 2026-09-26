import { useState, useRef, useEffect, useCallback } from 'react';
import {
  DiscoveredPoint2D,
  TrajectoryPoint,
  Waypoint,
  BotPose,
  RoverSweepState,
  GameVisualTheme,
  MapEnvironmentPreset,
  ExplorationStats,
  RouteStats,
  SerialErrorInfo,
} from '../types/worldDiscoverer';
import { simulateSonarPing, WORLD_PRESETS } from '../utils/worldSimulator';
import { radarAudio } from '../utils/audioSynth';

export function useWorldDiscoverer() {
  // Connection state
  const [connectionMode, setConnectionMode] = useState<'simulator' | 'serial'>('simulator');
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [portName, setPortName] = useState<string>('');
  const [baudRate, setBaudRate] = useState<number>(115200);
  const [serialError, setSerialError] = useState<SerialErrorInfo | null>(null);

  // Active Environment & Theme
  const [preset, setPreset] = useState<MapEnvironmentPreset>('dungeon_chamber');
  const [visualTheme, setVisualTheme] = useState<GameVisualTheme>('tactical_radar');
  const [isMuted, setIsMuted] = useState(false);

  // Bot Navigation & Trajectory State
  const initialPresetStart = WORLD_PRESETS.dungeon_chamber.recommendedStart;
  const [botPose, setBotPose] = useState<BotPose>({
    x: initialPresetStart.x,
    y: initialPresetStart.y,
    heading: initialPresetStart.heading, // 90° is facing North (+Y)
    speed: 0,
    isMoving: false,
    driveMode: 'manual',
    totalDistanceCm: 0,
  });

  // Recorded Bot Trajectory (Route Breadcrumbs)
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([
    {
      id: `t-start-${Date.now()}`,
      x: initialPresetStart.x,
      y: initialPresetStart.y,
      heading: initialPresetStart.heading,
      timestamp: Date.now(),
      distanceFromStartCm: 0,
      speed: 0,
      isWaypoint: true,
      waypointLabel: 'Inicio / Base',
    },
  ]);

  // Waypoints marked along route
  const [waypoints, setWaypoints] = useState<Waypoint[]>([
    {
      id: 'wp-start',
      x: initialPresetStart.x,
      y: initialPresetStart.y,
      label: 'Base / Despliegue',
      timestamp: Date.now(),
      color: '#10b981',
      icon: 'flag',
    },
  ]);

  // Autonomous exploration state
  const [isAutonomous, setIsAutonomous] = useState(false);

  // Rover & Sweep State (Sensor starts at 90°, normal sweep 0 to 30° / 0 to -30°, reduced to 0 to 10° / 0 to -10° on obstacle <= 40cm, PWM 175-198)
  const [roverState, setRoverState] = useState<RoverSweepState>({
    currentAngle: 90, // Punto de inicio: 90° (0° relativo / Frente)
    relativeAngle: 0,
    targetAngle: 90,
    sweepDirection: 'forward',
    isScanning: true,
    scanMode: 'normal_sweep', // Modo inicial normal: de 0 a 30° y de 0 a -30° (60° a 120° servo)
    scanSpanDeg: 30, // Amplitud normal de 30° (±30° relativo)
    isObstacleDetected: false,
    obstacleDistanceCm: null,
    obstacleDetectedAngle: null,
    obstacleThresholdCm: 40, // Umbral crítico de 40 cm
    surveyMinAngle: null,
    surveyMaxAngle: null,
    surveyStepDirection: 1,
    surveyPassesCount: 0,
    sweepPeriodSeconds: 20,
    elapsedInSweepSeconds: 0,
    totalSweepsCompleted: 0,
    currentDistanceCm: 120,
    maxRangeCm: 300,
    motorPwm: 185, // PWM por defecto dentro del rango calibrado 175 a 198
  });

  // Motor PWM state (calibrated between 175 and 198)
  const [motorPwm, setMotorPwmState] = useState<number>(185);

  const setMotorPwm = useCallback((pwm: number) => {
    const clamped = Math.max(175, Math.min(198, Math.round(pwm)));
    setMotorPwmState(clamped);
    setRoverState((prev) => ({ ...prev, motorPwm: clamped }));
    sendSerialCommand(`PWM:${clamped}`);
  }, []);

  // Discovered World Obstacles (Projected into Global World Coordinates)
  const [points, setPoints] = useState<DiscoveredPoint2D[]>([]);

  // Telemetry logs
  const [logs, setLogs] = useState<{ time: string; text: string; dir: 'in' | 'out' | 'sys' }[]>([]);

  // Web Serial refs
  const serialPortRef = useRef<any>(null);
  const serialReaderRef = useRef<any>(null);
  const keepReadingRef = useRef(false);

  // Timing refs for 20s sweep and simulation navigation
  const sweepIntervalRef = useRef<any>(null);
  const autonomousNavIntervalRef = useRef<any>(null);
  const lastPingSoundTimeRef = useRef<number>(0);
  const moveTimerRef = useRef<any>(null);
  const botPoseRef = useRef<BotPose>(botPose);
  botPoseRef.current = botPose;

  const trajectoryRef = useRef<TrajectoryPoint[]>(trajectory);
  trajectoryRef.current = trajectory;

  // Add log helper
  const addLog = useCallback((text: string, dir: 'in' | 'out' | 'sys' = 'sys') => {
    const now = new Date();
    const time =
      now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
    setLogs((prev) => [...prev.slice(-140), { time, text, dir }]);
  }, []);

  // Web Audio mute toggle
  const toggleMute = useCallback(() => {
    setIsMuted((prev) => {
      const next = !prev;
      radarAudio.isMuted = next;
      return next;
    });
  }, []);

  // Send command to Serial
  const sendSerialCommand = useCallback(
    async (cmd: string) => {
      addLog(cmd, 'out');
      if (serialPortRef.current && isConnected && serialPortRef.current.writable) {
        try {
          const writer = serialPortRef.current.writable.getWriter();
          const encoder = new TextEncoder();
          await writer.write(encoder.encode(cmd + '\n'));
          writer.releaseLock();
        } catch (err: any) {
          addLog(`Error al enviar: ${err?.message || 'Fallo de escritura'}`, 'sys');
        }
      }
    },
    [isConnected, addLog]
  );

  // Record a detected point into the Global 2D World Map (Only if < 70cm, and clears phantom walls if distance increased)
  const recordPoint = useCallback(
    (
      sensorAngleDeg: number,
      distanceCm: number,
      classification?: 'wall' | 'obstacle' | 'anomaly',
      providedWorldCoord?: { worldX: number; worldY: number }
    ) => {
      const currentBot = botPoseRef.current;
      const now = Date.now();

      // Sound feedback throttled
      if (now - lastPingSoundTimeRef.current > 350) {
        radarAudio.playSonarPing(sensorAngleDeg, distanceCm);
        lastPingSoundTimeRef.current = now;
      }

      // Beam angle in world coordinate system
      const offsetAngle = sensorAngleDeg - 90;
      const beamAngleDeg = currentBot.heading + offsetAngle;
      const beamRad = (beamAngleDeg * Math.PI) / 180;

      setPoints((prev) => {
        // REGLA: "como quedan dibujadas las paredes sin borrarse"
        // Las paredes descubiertas permanecen dibujadas permanentemente en el mapa sin borrarse.
        if (distanceCm >= 70.0) {
          return prev;
        }

        // Calcular coordenadas globales exactas para la pared a distancia < 70 cm
        let worldX: number;
        let worldY: number;

        if (providedWorldCoord) {
          worldX = providedWorldCoord.worldX;
          worldY = providedWorldCoord.worldY;
        } else {
          worldX = Number((currentBot.x + distanceCm * Math.cos(beamRad)).toFixed(1));
          worldY = Number((currentBot.y + distanceCm * Math.sin(beamRad)).toFixed(1));
        }

        // Spatial clustering: reforzar si está dentro de 7cm sin borrar otros puntos
        const existingIdx = prev.findIndex(
          (p) => Math.hypot(p.worldX - worldX, p.worldY - worldY) < 7.0
        );

        if (existingIdx >= 0) {
          const updated = [...prev];
          const item = updated[existingIdx];
          updated[existingIdx] = {
            ...item,
            hits: item.hits + 1,
            timestamp: now,
            distanceCm,
            worldX,
            worldY,
            type: classification || item.type,
          };
          return updated;
        }

        const newPoint: DiscoveredPoint2D = {
          id: `p-${now}-${Math.floor(Math.random() * 10000)}`,
          worldX,
          worldY,
          robotX: currentBot.x,
          robotY: currentBot.y,
          robotHeading: currentBot.heading,
          sensorAngle: sensorAngleDeg,
          distanceCm,
          timestamp: now,
          sweepCycle: roverState.totalSweepsCompleted,
          hits: 1,
          type: classification || (distanceCm < 40 ? 'obstacle' : 'wall'),
        };

        if (prev.length > 1200) {
          return [...prev.slice(prev.length - 1199), newPoint];
        }
        return [...prev, newPoint];
      });
    },
    [roverState.totalSweepsCompleted, addLog]
  );

  // Append a point to the bot's trajectory breadcrumb trail
  const recordTrajectoryBreadcrumb = useCallback(
    (newPose: BotPose, forced: boolean = false, label?: string) => {
      const prevPoints = trajectoryRef.current;
      const lastPoint = prevPoints[prevPoints.length - 1];

      if (lastPoint) {
        const distFromLast = Math.hypot(newPose.x - lastPoint.x, newPose.y - lastPoint.y);
        const headingDiff = Math.abs(newPose.heading - lastPoint.heading);

        // Record if moved more than 4cm, turned more than 10 degrees, or forced
        if (!forced && distFromLast < 4.0 && headingDiff < 10) {
          return;
        }
      }

      const totalDist = newPose.totalDistanceCm;
      const newPoint: TrajectoryPoint = {
        id: `t-${Date.now()}-${Math.floor(Math.random() * 1000)}`,
        x: Number(newPose.x.toFixed(1)),
        y: Number(newPose.y.toFixed(1)),
        heading: Number(newPose.heading.toFixed(1)),
        timestamp: Date.now(),
        distanceFromStartCm: Number(totalDist.toFixed(1)),
        speed: newPose.speed,
        isWaypoint: Boolean(label),
        waypointLabel: label,
      };

      setTrajectory((prev) => [...prev, newPoint]);
    },
    []
  );



  // Bot Navigation Engine (Drive Controls)
  const moveBot = useCallback(
    (deltaForwardCm: number, deltaHeadingDeg: number, speedCmS: number = 20, durationMs: number = 700) => {
      setBotPose((prev) => {
        const rad = (prev.heading * Math.PI) / 180;
        const newX = prev.x + deltaForwardCm * Math.cos(rad);
        const newY = prev.y + deltaForwardCm * Math.sin(rad);
        let newHeading = (prev.heading + deltaHeadingDeg) % 360;
        if (newHeading < 0) newHeading += 360;

        const distanceTraveled = Math.abs(deltaForwardCm);
        const nextPose: BotPose = {
          x: Number(newX.toFixed(1)),
          y: Number(newY.toFixed(1)),
          heading: Number(newHeading.toFixed(1)),
          speed: speedCmS,
          isMoving: deltaForwardCm !== 0 || deltaHeadingDeg !== 0,
          driveMode: isAutonomous ? 'autonomous' : 'manual',
          totalDistanceCm: prev.totalDistanceCm + distanceTraveled,
        };

        recordTrajectoryBreadcrumb(nextPose);
        return nextPose;
      });

      // Clear previous timeout and set speed/moving to 0 after driving pulse finishes
      if (moveTimerRef.current) clearTimeout(moveTimerRef.current);
      if (speedCmS > 0) {
        moveTimerRef.current = setTimeout(() => {
          setBotPose((prev) => ({ ...prev, isMoving: false, speed: 0 }));
        }, Math.max(600, durationMs));
      }
    },
    [isAutonomous, recordTrajectoryBreadcrumb]
  );

  // Cuando gire el bot, vuelve al estado normal de 0 a 30 y de 0 a -30
  const resetScanToNormal = useCallback(() => {
    setRoverState((prev) => ({
      ...prev,
      isScanning: true,
      scanMode: 'normal_sweep',
      scanSpanDeg: 30, // Rango normal: 0 a 30° y 0 a -30° (60° a 120° servo)
      isObstacleDetected: false,
      obstacleDistanceCm: null,
      obstacleDetectedAngle: null,
    }));
    sendSerialCommand('START');
    addLog('Giro del bot detectado: Senso restablecido al estado normal (0 a 30° y 0 a -30°).', 'sys');
  }, [sendSerialCommand, addLog]);

  // Rutina de avance y retroceso modificada (duración y potencia calibradas para mover los motores)
  const driveForward = useCallback(
    (distCm: number = 20, customPwm?: number) => {
      const activePwm = customPwm !== undefined ? Math.max(175, Math.min(198, customPwm)) : motorPwm;
      const speedCmS = Number(((activePwm / 185) * 25).toFixed(1));
      const durationMs = Math.max(450, Math.round(distCm * 25));
      moveBot(distCm, 0, speedCmS, durationMs);
      sendSerialCommand(`MOVE:F,${activePwm},${durationMs}`);
      addLog(
        `Avanzando paso de +${distCm} cm hacia rumbo ${Math.round(botPoseRef.current.heading)}° (PWM: ${activePwm}, duración: ${durationMs}ms).`,
        'sys'
      );
    },
    [moveBot, sendSerialCommand, motorPwm, addLog]
  );

  const driveBackward = useCallback(
    (distCm: number = 20, customPwm?: number) => {
      const activePwm = customPwm !== undefined ? Math.max(175, Math.min(198, customPwm)) : motorPwm;
      const speedCmS = Number(((activePwm / 185) * 20).toFixed(1));
      const durationMs = Math.max(450, Math.round(distCm * 25));
      moveBot(-distCm, 0, speedCmS, durationMs);
      sendSerialCommand(`MOVE:B,${activePwm},${durationMs}`);
      addLog(
        `Retrocediendo paso de -${distCm} cm desde rumbo ${Math.round(botPoseRef.current.heading)}° (PWM: ${activePwm}, duración: ${durationMs}ms).`,
        'sys'
      );
    },
    [moveBot, sendSerialCommand, motorPwm, addLog]
  );

  // Giros: conservados como en la versión anterior + restablecimiento del senso a 0 a 30° y 0 a -30°
  const turnLeft = useCallback(
    (deg: number = 15, customPwm?: number) => {
      const activePwm = customPwm !== undefined ? Math.max(175, Math.min(198, customPwm)) : motorPwm;
      moveBot(0, deg, 15, 300);
      sendSerialCommand(`TURN:L,${activePwm},${deg}`);
      resetScanToNormal(); // Al girar el bot, vuelve al estado normal de 0 a 30 y de 0 a -30
    },
    [moveBot, sendSerialCommand, motorPwm, resetScanToNormal]
  );

  const turnRight = useCallback(
    (deg: number = 15, customPwm?: number) => {
      const activePwm = customPwm !== undefined ? Math.max(175, Math.min(198, customPwm)) : motorPwm;
      moveBot(0, -deg, 15, 300);
      sendSerialCommand(`TURN:R,${activePwm},${deg}`);
      resetScanToNormal(); // Al girar el bot, vuelve al estado normal de 0 a 30 y de 0 a -30
    },
    [moveBot, sendSerialCommand, motorPwm, resetScanToNormal]
  );

  const stopBot = useCallback(() => {
    if (moveTimerRef.current) clearTimeout(moveTimerRef.current);
    setBotPose((prev) => ({ ...prev, speed: 0, isMoving: false }));
    sendSerialCommand('STOP');
  }, [sendSerialCommand]);

  // Toggle Autonomous Exploration Mode
  const toggleAutonomous = useCallback(() => {
    setIsAutonomous((prev) => {
      const next = !prev;
      addLog(
        next
          ? 'Exploración Autónoma del Bot ACTIVADA (Navegación y Registro Continuo).'
          : 'Exploración Autónoma PAUSADA.',
        'sys'
      );
      if (!next) {
        stopBot();
      }
      return next;
    });
  }, [addLog, stopBot]);

  // Autonomous bot drive loop (runs when isAutonomous is true)
  useEffect(() => {
    if (!isAutonomous) {
      if (autonomousNavIntervalRef.current) clearInterval(autonomousNavIntervalRef.current);
      return;
    }

    autonomousNavIntervalRef.current = setInterval(() => {
      const currentBot = botPoseRef.current;
      const currentDist = roverState.currentDistanceCm;
      const currentSensorAngle = roverState.currentAngle;

      // Obstacle avoidance logic based on 40cm trigger threshold
      const isObstacleAhead = currentDist <= 40.0;

      if (isObstacleAhead) {
        // When obstacle is detected within 40cm, steer away and reset scan range on turn
        const turnDirection = currentSensorAngle >= 90 ? -18 : 18;
        moveBot(0.5, turnDirection, 8, 300);
        resetScanToNormal();
      } else {
        // Forward exploration cruise with smooth organic wander
        const gentleTurn = (Math.random() - 0.5) * 3;
        moveBot(3.5, gentleTurn, 16, 250);
      }
    }, 180);

    return () => {
      if (autonomousNavIntervalRef.current) clearInterval(autonomousNavIntervalRef.current);
    };
  }, [isAutonomous, roverState.currentDistanceCm, roverState.currentAngle, moveBot, resetScanToNormal]);

  // Keyboard navigation listener (W, A, S, D, Arrows, Space)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Ignore if user is typing in an input or textarea
      const target = e.target as HTMLElement;
      if (target.tagName === 'INPUT' || target.tagName === 'TEXTAREA') return;

      switch (e.key) {
        case 'w':
        case 'W':
        case 'ArrowUp':
          e.preventDefault();
          driveForward(20);
          break;
        case 's':
        case 'S':
        case 'ArrowDown':
          e.preventDefault();
          driveBackward(20);
          break;
        case 'a':
        case 'A':
        case 'ArrowLeft':
          e.preventDefault();
          turnLeft(15);
          break;
        case 'd':
        case 'D':
        case 'ArrowRight':
          e.preventDefault();
          turnRight(15);
          break;
        case ' ':
          e.preventDefault();
          stopBot();
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [driveForward, driveBackward, turnLeft, turnRight, stopBot]);

  // Add custom Waypoint at current bot position
  const handleAddWaypoint = useCallback(
    (label?: string) => {
      const current = botPoseRef.current;
      const name = label || `Hito ${waypoints.length + 1} (${Math.round(current.x)}, ${Math.round(current.y)})`;
      const newWp: Waypoint = {
        id: `wp-${Date.now()}`,
        x: current.x,
        y: current.y,
        label: name,
        timestamp: Date.now(),
        color: '#38bdf8',
        icon: 'pin',
      };
      setWaypoints((prev) => [...prev, newWp]);
      recordTrajectoryBreadcrumb(current, true, name);
      addLog(`Waypoint marcado en [${current.x}cm, ${current.y}cm]: "${name}"`, 'sys');
    },
    [waypoints.length, recordTrajectoryBreadcrumb, addLog]
  );

  // Reset route / trajectory
  const handleResetRoute = useCallback(() => {
    const presetStart = WORLD_PRESETS[preset]?.recommendedStart || { x: 0, y: 30, heading: 90 };
    const resetPose: BotPose = {
      x: presetStart.x,
      y: presetStart.y,
      heading: presetStart.heading,
      speed: 0,
      isMoving: false,
      driveMode: 'manual',
      totalDistanceCm: 0,
    };
    setBotPose(resetPose);
    setTrajectory([
      {
        id: `t-start-${Date.now()}`,
        x: resetPose.x,
        y: resetPose.y,
        heading: resetPose.heading,
        timestamp: Date.now(),
        distanceFromStartCm: 0,
        speed: 0,
        isWaypoint: true,
        waypointLabel: 'Inicio / Base',
      },
    ]);
    setWaypoints([
      {
        id: 'wp-start',
        x: resetPose.x,
        y: resetPose.y,
        label: 'Base / Despliegue',
        timestamp: Date.now(),
        color: '#10b981',
      },
    ]);
    addLog('Trayecto del bot reiniciado a la posición de inicio.', 'sys');
  }, [preset, addLog]);

  // Reset all mapped world points
  const handleClearWorldMap = useCallback(() => {
    setPoints([]);
    addLog('Puntos de obstáculos del mapa borrados.', 'sys');
  }, [addLog]);

  // Reset everything (route + map points)
  const handleClearAll = useCallback(() => {
    handleResetRoute();
    handleClearWorldMap();
    addLog('Mapa y trayecto completamente reseteados.', 'sys');
  }, [handleResetRoute, handleClearWorldMap, addLog]);

  // Change Preset
  const handleSelectPreset = useCallback(
    (newPreset: MapEnvironmentPreset) => {
      setPreset(newPreset);
      const conf = WORLD_PRESETS[newPreset];
      if (conf) {
        const start = conf.recommendedStart;
        const newPose: BotPose = {
          x: start.x,
          y: start.y,
          heading: start.heading,
          speed: 0,
          isMoving: false,
          driveMode: 'manual',
          totalDistanceCm: 0,
        };
        setBotPose(newPose);
        setTrajectory([
          {
            id: `t-start-${Date.now()}`,
            x: start.x,
            y: start.y,
            heading: start.heading,
            timestamp: Date.now(),
            distanceFromStartCm: 0,
            speed: 0,
            isWaypoint: true,
            waypointLabel: 'Inicio / Base',
          },
        ]);
        setWaypoints([
          {
            id: 'wp-start',
            x: start.x,
            y: start.y,
            label: 'Base / Despliegue',
            timestamp: Date.now(),
            color: '#10b981',
          },
        ]);
        setPoints([]);
        addLog(`Cargado entorno: ${conf.name}`, 'sys');
      }
    },
    [addLog]
  );

  // Manual Servo Angle Controls
  const handleTurnLeft = useCallback(() => {
    setRoverState((prev) => {
      const nextAngle = Math.min(180, prev.currentAngle + 10);
      sendSerialCommand(`GOTO:${nextAngle}`);
      return { ...prev, currentAngle: nextAngle, targetAngle: nextAngle };
    });
  }, [sendSerialCommand]);

  const handleTurnRight = useCallback(() => {
    setRoverState((prev) => {
      const nextAngle = Math.max(0, prev.currentAngle - 10);
      sendSerialCommand(`GOTO:${nextAngle}`);
      return { ...prev, currentAngle: nextAngle, targetAngle: nextAngle };
    });
  }, [sendSerialCommand]);

  const handleGotoAngle = useCallback(
    (angle: number) => {
      const clamped = Math.max(0, Math.min(180, angle));
      setRoverState((prev) => ({ ...prev, currentAngle: clamped, targetAngle: clamped }));
      sendSerialCommand(`GOTO:${clamped}`);
    },
    [sendSerialCommand]
  );

  const handleToggleScan = useCallback(() => {
    setRoverState((prev) => {
      const nextState = !prev.isScanning;
      if (nextState) {
        sendSerialCommand('START');
      } else {
        sendSerialCommand('STOP');
      }
      return { ...prev, isScanning: nextState };
    });
  }, [sendSerialCommand]);

  // Read lines from Serial Port using non-blocking reader pattern
  const readSerialLoop = useCallback(
    async (port: any) => {
      let buffer = '';
      const textDecoder = new TextDecoder();

      while (keepReadingRef.current && port.readable) {
        try {
          const reader = port.readable.getReader();
          serialReaderRef.current = reader;

          while (keepReadingRef.current) {
            const { value, done } = await reader.read();
            if (done) break;

            if (value) {
              buffer += textDecoder.decode(value, { stream: true });
              const lines = buffer.split('\n');
              buffer = lines.pop() || '';

              for (const rawLine of lines) {
                const line = rawLine.trim();
                if (!line) continue;
                addLog(line, 'in');

                // Check for PING:angle,distance or DIST:distance (Synchronized with Zero Phase Lag)
                if (line.startsWith('PING:')) {
                  const parts = line.substring(5).split(',');
                  if (parts.length >= 2) {
                    const angle = parseFloat(parts[0]);
                    const dist = parseFloat(parts[1]);
                    if (!isNaN(angle) && !isNaN(dist)) {
                      const relAngle = Math.round(angle - 90);
                      const isObstacle = dist <= 40.0;

                      setRoverState((prev) => {
                        let nextMode = prev.scanMode;
                        let nextSpan = prev.scanSpanDeg || 30;
                        let obsDetectedAngle = prev.obstacleDetectedAngle;

                        if (isObstacle) {
                          // "no quiero que pares si encuentra el obstaculo, solo quiero que disminuyas el rango de senso,
                          // pasa de 0 a 30 y 0 a -30 a 0 a 10 y de 0 a -10"
                          if (prev.scanMode !== 'obstacle_reduced_sweep') {
                            nextMode = 'obstacle_reduced_sweep';
                            nextSpan = 10;
                            obsDetectedAngle = relAngle;
                            radarAudio.playObstacleAlert();
                            addLog(
                              `¡Obstáculo detectado a ${dist.toFixed(1)} cm (≤ 40 cm)! Rango de senso reducido a 0° a 10° y 0° a -10° (muestreo continuo activo).`,
                              'sys'
                            );
                          }
                        }

                        return {
                          ...prev,
                          isScanning: true, // MUESTREO CONTINUO (NO SE DETIENE)
                          currentAngle: Math.round(angle),
                          relativeAngle: relAngle,
                          currentDistanceCm: dist,
                          isObstacleDetected: isObstacle,
                          obstacleDistanceCm: isObstacle ? dist : prev.obstacleDistanceCm,
                          obstacleDetectedAngle: obsDetectedAngle,
                          scanMode: nextMode,
                          scanSpanDeg: nextSpan,
                        };
                      });

                      recordPoint(angle, dist);
                    }
                  }
                } else if (line.startsWith('DIST:')) {
                  const distVal = parseFloat(line.substring(5).trim());
                  if (!isNaN(distVal)) {
                    setRoverState((prev) => ({
                      ...prev,
                      currentDistanceCm: distVal,
                      isObstacleDetected: distVal <= 40.0 || prev.isObstacleDetected,
                      obstacleDistanceCm: distVal <= 40.0 ? distVal : prev.obstacleDistanceCm,
                    }));
                  }
                } else if (line.startsWith('PWM:')) {
                  const pwmVal = parseInt(line.substring(4).trim(), 10);
                  if (!isNaN(pwmVal) && pwmVal >= 175 && pwmVal <= 198) {
                    setMotorPwmState(pwmVal);
                    setRoverState((prev) => ({ ...prev, motorPwm: pwmVal }));
                  }
                } else if (line.startsWith('POS:')) {
                  // Real odometry position from ESP32: POS:x,y,heading
                  if (connectionMode === 'serial') {
                    const parts = line.substring(4).split(',');
                    if (parts.length >= 3) {
                      const rx = parseFloat(parts[0]);
                      const ry = parseFloat(parts[1]);
                      const rheading = parseFloat(parts[2]);
                      if (!isNaN(rx) && !isNaN(ry) && !isNaN(rheading)) {
                        setBotPose((prev) => {
                          // Evitar volver a la posición original (0, 30) si el bot ya ha avanzado
                          if (Math.hypot(rx, ry - 30) < 1.0 && prev.totalDistanceCm > 5.0) {
                            return prev;
                          }
                          const distInc = Math.hypot(rx - prev.x, ry - prev.y);
                          const nextPose: BotPose = {
                            ...prev,
                            x: rx,
                            y: ry,
                            heading: rheading,
                            totalDistanceCm: prev.totalDistanceCm + distInc,
                          };
                          recordTrajectoryBreadcrumb(nextPose);
                          return nextPose;
                        });
                      }
                    }
                  }
                } else if (line.includes('SWEEP_CYCLE_COMPLETE')) {
                  radarAudio.playSweepCycleComplete();
                  setRoverState((prev) => ({
                    ...prev,
                    totalSweepsCompleted: prev.totalSweepsCompleted + 1,
                  }));
                }
              }
            }
          }
          reader.releaseLock();
          serialReaderRef.current = null;
        } catch (err: any) {
          if (keepReadingRef.current) {
            addLog(`Lectura serial interrumpida: ${err.message}`, 'sys');
          }
          break;
        }
      }
    },
    [addLog, recordPoint, recordTrajectoryBreadcrumb]
  );

  // Connect Serial Port
  const handleConnectSerial = useCallback(async () => {
    if (!('serial' in navigator)) {
      setSerialError({
        code: 'UNSUPPORTED',
        title: 'Navegador sin Web Serial API',
        message: 'Tu navegador no soporta Web Serial API. Usa Google Chrome, Edge u Opera en PC/Mac.',
        reasons: ['Navegador incompatible (Safari, Firefox no tienen Web Serial activado por defecto)'],
        solutions: ['Abre esta aplicación en Google Chrome o Microsoft Edge.'],
      });
      return;
    }

    try {
      setIsConnecting(true);
      setSerialError(null);
      addLog('Solicitando puerto serial al sistema...', 'sys');

      const port = await (navigator as any).serial.requestPort();
      serialPortRef.current = port;

      try {
        await port.open({ baudRate });
      } catch (openErr: any) {
        const errorMsg = openErr?.message || '';
        if (errorMsg.includes('Failed to open serial port') || openErr.name === 'InvalidStateError') {
          setSerialError({
            code: 'LOCKED_OR_IN_USE',
            title: 'Puerto COM Ocupado o Bloqueado',
            message:
              'El puerto serial del ESP32 está siendo utilizado por otro programa en tu ordenador.',
            reasons: [
              'El Monitor Serie de Arduino IDE o PlatformIO está ABIERTO.',
              'Otra pestaña de navegador tiene la conexión abierta.',
              'El driver USB (CH340 / CP2102) quedó en estado suspendido.',
            ],
            solutions: [
              '1. En Arduino IDE, CIERRA la ventana del "Monitor Serie".',
              '2. Desconecta y vuelve a conectar el cable USB del ESP32.',
              '3. O pulsa el botón "Usar Modo Simulador" para descubrir mundos virtuales.',
            ],
          });
          setIsConnecting(false);
          return;
        }
        throw openErr;
      }

      const info = port.getInfo ? port.getInfo() : {};
      const name = info.usbVendorId
        ? `USB (${info.usbVendorId.toString(16)}:${info.usbProductId?.toString(16) || ''})`
        : 'Puerto Serial ESP32';

      setPortName(name);
      setIsConnected(true);
      setConnectionMode('serial');
      addLog(`Conectado exitosamente a ${name} @ ${baudRate} bps`, 'sys');

      keepReadingRef.current = true;
      readSerialLoop(port);

      // Start ESP32 scanning
      sendSerialCommand('START');
    } catch (err: any) {
      if (err.name !== 'NotFoundError') {
        setSerialError({
          code: 'UNKNOWN',
          title: 'Error al conectar puerto serial',
          message: err.message || 'No se pudo establecer la conexión.',
          reasons: ['Permiso denegado por el usuario o cable USB desconectado.'],
          solutions: ['Verifica el cable USB y vuelve a intentarlo.'],
        });
      }
    } finally {
      setIsConnecting(false);
    }
  }, [baudRate, addLog, readSerialLoop, sendSerialCommand]);

  // Disconnect Serial
  const handleDisconnectSerial = useCallback(async () => {
    keepReadingRef.current = false;
    addLog('Desconectando puerto serial...', 'sys');

    if (serialReaderRef.current) {
      try {
        await serialReaderRef.current.cancel();
        serialReaderRef.current.releaseLock();
      } catch {}
      serialReaderRef.current = null;
    }

    if (serialPortRef.current) {
      try {
        await serialPortRef.current.close();
      } catch {}
      serialPortRef.current = null;
    }

    setIsConnected(false);
    setPortName('');
    addLog('Puerto serial desconectado.', 'sys');
  }, [addLog]);

  // Switch to simulator
  const switchToSimulator = useCallback(() => {
    if (isConnected) {
      handleDisconnectSerial();
    }
    setConnectionMode('simulator');
    setSerialError(null);
    addLog('Modo simulador 2D activado.', 'sys');
  }, [isConnected, handleDisconnectSerial, addLog]);

  // Smart Sonar Sweep Loop (Starts at 90°, probes ±15°, focused survey [+25°, -10°] on obstacle <= 40cm, max bounds ±55°)
  useEffect(() => {
    if (!roverState.isScanning) {
      if (sweepIntervalRef.current) clearInterval(sweepIntervalRef.current);
      return;
    }

    const STEP_INTERVAL_MS = 80;

    sweepIntervalRef.current = setInterval(() => {
      setRoverState((prev) => {
        const CENTER_ANGLE = 90; // 0° relativo (Frente)
        const NORMAL_SPAN = 30; // Estado normal: de 0 a 30° y de 0 a -30° (60° a 120° servo)
        const OBSTACLE_SPAN = 10; // Con obstáculo: de 0 a 10° y de 0 a -10° (80° a 100° servo)
        const THRESHOLD_CM = prev.obstacleThresholdCm || 40; // 40 cm

        // 1. Muestreo sincronizado sin desfase en el ángulo actual
        const currentSampleAngle = prev.currentAngle;
        let newDist = prev.currentDistanceCm;
        if (connectionMode === 'simulator') {
          const sim = simulateSonarPing(botPoseRef.current, currentSampleAngle, preset, prev.maxRangeCm);
          newDist = Number(sim.distanceCm.toFixed(1));
          recordPoint(currentSampleAngle, newDist, sim.type, {
            worldX: sim.worldX,
            worldY: sim.worldY,
          });
        }

        let newAngle = currentSampleAngle;
        let newDir = prev.sweepDirection;
        let newMode = prev.scanMode;
        let activeSpan = prev.scanSpanDeg || NORMAL_SPAN;
        let newSweeps = prev.totalSweepsCompleted;
        let isObstacle = prev.isObstacleDetected;
        let obstacleDist = prev.obstacleDistanceCm;
        let obsDetectedAngle = prev.obstacleDetectedAngle;

        // 2. Evaluación de obstáculo:
        // "no quiero que pares si encuentra el obstaculo, solo quiero que disminuyas el rango de senso,
        //  pasa de 0 a 30 y 0 a -30 a 0 a 10 y de 0 a -10"
        if (newDist <= THRESHOLD_CM) {
          isObstacle = true;
          obstacleDist = newDist;
          obsDetectedAngle = Math.round(currentSampleAngle - CENTER_ANGLE);
          newMode = 'obstacle_reduced_sweep';
          activeSpan = OBSTACLE_SPAN; // Rango reducido a 10° (80° a 100°)
        } else if (newMode === 'obstacle_reduced_sweep' && newDist > THRESHOLD_CM + 15) {
          // Si el camino se despejó ampliamente y no hay obstáculo
          isObstacle = false;
        }

        // 3. Barrido oscilatorio continuo dentro de activeSpan
        const minAngle = CENTER_ANGLE - activeSpan; // 60° (normal) u 80° (obstáculo)
        const maxAngle = CENTER_ANGLE + activeSpan; // 120° (normal) o 100° (obstáculo)
        const stepDeg = 1.8;

        if (newDir === 'forward') {
          newAngle += stepDeg;
          if (newAngle >= maxAngle) {
            newAngle = maxAngle;
            newDir = 'backward';
            newSweeps += 1;
          }
        } else {
          newAngle -= stepDeg;
          if (newAngle <= minAngle) {
            newAngle = minAngle;
            newDir = 'forward';
            newSweeps += 1;
          }
        }

        // Si el ángulo estaba fuera del nuevo rango (por ejemplo, al reducirse el span repentinamente)
        if (newAngle > maxAngle) {
          newAngle = maxAngle;
          newDir = 'backward';
        }
        if (newAngle < minAngle) {
          newAngle = minAngle;
          newDir = 'forward';
        }

        const relativeAngle = Math.round(newAngle - CENTER_ANGLE);
        const progressFrac = Math.abs(newAngle - minAngle) / Math.max(1, maxAngle - minAngle);
        const elapsedSec = Number((progressFrac * prev.sweepPeriodSeconds).toFixed(1));

        return {
          ...prev,
          isScanning: true, // MUESTREO NUNCA SE DETIENE POR OBSTÁCULO
          currentAngle: Number(newAngle.toFixed(1)),
          relativeAngle,
          sweepDirection: newDir,
          scanMode: newMode,
          scanSpanDeg: activeSpan,
          isObstacleDetected: isObstacle,
          obstacleDistanceCm: obstacleDist,
          obstacleDetectedAngle: obsDetectedAngle,
          totalSweepsCompleted: newSweeps,
          elapsedInSweepSeconds: elapsedSec,
          currentDistanceCm: newDist,
        };
      });
    }, STEP_INTERVAL_MS);

    return () => {
      if (sweepIntervalRef.current) clearInterval(sweepIntervalRef.current);
    };
  }, [
    roverState.isScanning,
    roverState.sweepPeriodSeconds,
    connectionMode,
    preset,
    recordPoint,
    addLog,
  ]);

  // Exploration Statistics & Route Stats
  const totalDistM = Number((botPose.totalDistanceCm / 100).toFixed(2));
  const activeTimeSec = roverState.totalSweepsCompleted * 20 + roverState.elapsedInSweepSeconds;

  const explorationStats: ExplorationStats = {
    fogClearedPercentage: Math.min(100, Math.round(((points.length * 0.4) + (trajectory.length * 0.3)))),
    totalAreaM2: Number(((points.length * 0.15) + (totalDistM * 0.8)).toFixed(1)),
    obstaclesDetectedCount: points.length,
    activeScanTimeSeconds: activeTimeSec,
    currentSectorName: WORLD_PRESETS[preset]?.name || 'Sector Desconocido',
    maxRangeReachedCm: roverState.maxRangeCm,
    anomaliesFound: points.filter((p) => p.type === 'anomaly').length,
    totalDistanceTraveledM: totalDistM,
    routePointsCount: trajectory.length,
  };

  const routeStats: RouteStats = {
    totalDistanceMeters: totalDistM,
    activeDriveTimeSeconds: activeTimeSec,
    pointsRecordedCount: trajectory.length,
    waypointsCount: waypoints.length,
    currentSpeedCmS: botPose.speed,
    averageSpeedCmS: activeTimeSec > 0 ? Number((botPose.totalDistanceCm / activeTimeSec).toFixed(1)) : 0,
    obstaclesDetectedCount: points.length,
    fogClearedPercentage: explorationStats.fogClearedPercentage,
    currentSectorName: explorationStats.currentSectorName,
    bounds: {
      minX: Math.min(WORLD_PRESETS[preset]?.bounds?.minX ?? -260, Math.round(botPose.x - 150)),
      maxX: Math.max(WORLD_PRESETS[preset]?.bounds?.maxX ?? 260, Math.round(botPose.x + 150)),
      minY: Math.min(WORLD_PRESETS[preset]?.bounds?.minY ?? -40, Math.round(botPose.y - 150)),
      maxY: Math.max(WORLD_PRESETS[preset]?.bounds?.maxY ?? 420, Math.round(botPose.y + 150)),
    },
  };

  return {
    connectionMode,
    setConnectionMode,
    isConnected,
    isConnecting,
    portName,
    baudRate,
    setBaudRate,
    serialError,
    dismissSerialError: () => setSerialError(null),
    switchToSimulator,
    preset,
    setPreset: handleSelectPreset,
    visualTheme,
    setVisualTheme,
    isMuted,
    toggleMute,
    botPose,
    trajectory,
    waypoints,
    isAutonomous,
    toggleAutonomous,
    driveForward,
    driveBackward,
    turnLeft,
    turnRight,
    stopBot,
    handleAddWaypoint,
    handleResetRoute,
    handleClearWorldMap,
    handleClearAll,
    roverState,
    setRoverState,
    points,
    logs,
    clearLogs: () => setLogs([]),
    explorationStats,
    routeStats,
    handleConnectSerial,
    handleDisconnectSerial,
    handleTurnLeft,
    handleTurnRight,
    handleGotoAngle,
    handleToggleScan,
    sendSerialCommand,
    motorPwm,
    setMotorPwm,
  };
}
