import { useState, useRef, useEffect, useCallback } from 'react';
import {
  Point3D,
  TurntableState,
  UltrasonicSensorData,
  ScannerSettings,
  ScanStatus,
  ConnectionMode,
} from '../types/scanner';
import { parseSerialLine, convertToCartesian } from '../utils/serialParser';
import {
  SimulatedModel,
  simulateUltrasonicPing,
} from '../utils/simulatedScanner';

export function useScannerHardware() {
  // Connection state
  const [connectionMode, setConnectionMode] = useState<ConnectionMode>('simulator');
  const [isConnected, setIsConnected] = useState(false);
  const [isConnecting, setIsConnecting] = useState(false);
  const [portName, setPortName] = useState<string>('');
  const [baudRate, setBaudRate] = useState<number>(115200);
  const [simulatedModel, setSimulatedModel] = useState<SimulatedModel>('vase');

  // Scanner status
  const [scanStatus, setScanStatus] = useState<ScanStatus>('idle');

  // Turntable State
  const [turntable, setTurntable] = useState<TurntableState>({
    currentAngle: 0,
    targetAngle: 0,
    direction: 'idle',
    stepSize: 2,
    speed: 30,
    isRotating: false,
  });

  // Sensor reading
  const [sensorData, setSensorData] = useState<UltrasonicSensorData>({
    distanceCm: 11.4,
    rawEchoMicros: 664,
    valid: true,
    lastPingTime: Date.now(),
  });

  // 3D Point Cloud array
  const [points, setPoints] = useState<Point3D[]>([]);

  // Settings
  const [settings, setSettings] = useState<ScannerSettings>({
    turntableRadius: 15.0,
    minDistance: 2.0,
    maxDistance: 14.5,
    angularResolution: 2,
    fullSweepAngle: 360,
    verticalLayers: 1,
    currentLayer: 0,
    layerHeightMm: 5.0,
    filterOutliers: true,
    samplesPerAngle: 3,
  });

  // Serial log history
  const [serialLogs, setSerialLogs] = useState<
    { time: string; text: string; direction: 'in' | 'out' | 'sys' }[]
  >([]);

  // Web Serial refs
  const serialPortRef = useRef<any>(null);
  const serialReaderRef = useRef<any>(null);
  const serialWriterRef = useRef<any>(null);
  const keepReadingRef = useRef(false);

  // Scan simulation interval ref
  const scanIntervalRef = useRef<any>(null);

  // Append serial log helper
  const addLog = useCallback(
    (text: string, direction: 'in' | 'out' | 'sys' = 'sys') => {
      const now = new Date();
      const time = now.toTimeString().split(' ')[0] + '.' + String(now.getMilliseconds()).padStart(3, '0');
      setSerialLogs((prev) => [...prev.slice(-150), { time, text, direction }]);
    },
    []
  );

  // Add a 3D point to cloud (deduplicating or adding)
  const addPoint = useCallback((pt: Point3D) => {
    setPoints((prev) => {
      // If we have an existing point within same angle & layer, update it or append
      return [...prev, pt];
    });
  }, []);

  // Web Serial: Send raw command string
  const sendSerialCommand = useCallback(
    async (cmd: string) => {
      addLog(cmd, 'out');

      if (connectionMode === 'serial') {
        if (serialWriterRef.current) {
          try {
            const encoder = new TextEncoder();
            const data = encoder.encode(cmd + '\n');
            await serialWriterRef.current.write(data);
          } catch (err) {
            console.error('Serial write error:', err);
            addLog(`Error al enviar: ${err}`, 'sys');
          }
        } else {
          addLog('Error: Puerto Serial no disponible para escritura', 'sys');
        }
      } else if (connectionMode === 'simulator') {
        // Handle in software simulator
        handleSimulatedCommand(cmd);
      }
    },
    [connectionMode, addLog]
  );

  // Connect Web Serial
  const connectSerial = useCallback(async () => {
    if (typeof navigator === 'undefined' || !('serial' in navigator)) {
      alert('La Web Serial API no está soportada en este navegador. Utiliza Google Chrome, Microsoft Edge u Opera.');
      return;
    }

    setIsConnecting(true);
    addLog(`Solicitando puerto serial a ${baudRate} baudios...`, 'sys');

    try {
      const port = await (navigator as any).serial.requestPort();
      await port.open({ baudRate });

      serialPortRef.current = port;
      const portInfo = port.getInfo();
      const name = portInfo.usbVendorId
        ? `USB VID:${portInfo.usbVendorId.toString(16)}`
        : 'Puerto Serie ESP32';
      setPortName(name);
      setIsConnected(true);
      addLog(`ESP32 Conectado con éxito (${name})`, 'sys');

      // Setup writer
      const textEncoder = new TextEncoderStream();
      textEncoder.readable.pipeTo(port.writable);
      serialWriterRef.current = textEncoder.writable.getWriter();

      // Setup reader loop
      keepReadingRef.current = true;
      const textDecoder = new TextDecoderStream();
      port.readable.pipeTo(textDecoder.writable);
      const reader = textDecoder.readable.getReader();
      serialReaderRef.current = reader;

      let buffer = '';
      (async () => {
        try {
          while (keepReadingRef.current) {
            const { value, done } = await reader.read();
            if (done) break;
            if (value) {
              buffer += value;
              const lines = buffer.split('\n');
              buffer = lines.pop() || '';

              for (const line of lines) {
                const trimmed = line.trim();
                if (!trimmed) continue;
                addLog(trimmed, 'in');

                const parsed = parseSerialLine(trimmed);
                if (parsed) {
                  if (parsed.type === 'data' && parsed.angle !== undefined && parsed.distance !== undefined) {
                    setTurntable((prev) => ({ ...prev, currentAngle: parsed.angle! }));
                    setSensorData({
                      distanceCm: parsed.distance,
                      rawEchoMicros: Math.round(parsed.distance / 0.0343 * 2),
                      valid: parsed.distance > settings.minDistance && parsed.distance < settings.maxDistance,
                      lastPingTime: Date.now(),
                    });

                    const cartesian = convertToCartesian(
                      parsed.angle,
                      parsed.distance,
                      parsed.z ?? 0,
                      settings
                    );
                    if (cartesian) {
                      addPoint(cartesian);
                    }
                  } else if (parsed.type === 'status' && parsed.statusMessage) {
                    if (parsed.statusMessage.includes('SCAN_COMPLETE')) {
                      setScanStatus('completed');
                    } else if (parsed.statusMessage.includes('POS:')) {
                      const pos = parseFloat(parsed.statusMessage.split(':')[2] || '0');
                      setTurntable((prev) => ({ ...prev, currentAngle: pos }));
                    }
                  }
                }
              }
            }
          }
        } catch (readErr) {
          console.error('Serial read loop error:', readErr);
        }
      })();
    } catch (err: any) {
      console.error('Serial connection error:', err);
      addLog(`Fallo al conectar puerto serial: ${err.message || err}`, 'sys');
    } finally {
      setIsConnecting(false);
    }
  }, [baudRate, addLog, settings, addPoint]);

  // Disconnect Web Serial
  const disconnectSerial = useCallback(async () => {
    keepReadingRef.current = false;
    try {
      if (serialReaderRef.current) {
        await serialReaderRef.current.cancel();
        serialReaderRef.current = null;
      }
      if (serialWriterRef.current) {
        await serialWriterRef.current.close();
        serialWriterRef.current = null;
      }
      if (serialPortRef.current) {
        await serialPortRef.current.close();
        serialPortRef.current = null;
      }
    } catch (e) {
      console.warn('Error during disconnect:', e);
    }
    setIsConnected(false);
    setPortName('');
    addLog('Puerto Serial desconectado', 'sys');
  }, [addLog]);

  // Simulator connect/disconnect
  const connectSimulator = useCallback(() => {
    setIsConnected(true);
    setPortName('Simulador Virtual 3D');
    addLog('Simulador de hardware ESP32 y sensor ultrasónico iniciado', 'sys');

    // Run an initial ping
    const result = simulateUltrasonicPing(turntable.currentAngle, 0, simulatedModel, settings);
    setSensorData({
      distanceCm: result.distance,
      rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
      valid: true,
      lastPingTime: Date.now(),
    });
  }, [addLog, turntable.currentAngle, simulatedModel, settings]);

  const disconnectSimulator = useCallback(() => {
    if (scanIntervalRef.current) {
      clearInterval(scanIntervalRef.current);
      scanIntervalRef.current = null;
    }
    setIsConnected(false);
    setScanStatus('idle');
    addLog('Simulador de hardware detenido', 'sys');
  }, [addLog]);

  // Handle command when in simulator mode
  const handleSimulatedCommand = useCallback(
    (cmd: string) => {
      const upper = cmd.trim().toUpperCase();

      if (upper.startsWith('LEFT') || upper.startsWith('TL')) {
        let deg = turntable.stepSize;
        const parts = upper.split(':');
        if (parts[1]) deg = parseFloat(parts[1]) || turntable.stepSize;

        setTurntable((prev) => {
          const nextAngle = Math.max(0, prev.currentAngle - deg);
          const result = simulateUltrasonicPing(nextAngle, 0, simulatedModel, settings);
          setSensorData({
            distanceCm: result.distance,
            rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
            valid: true,
            lastPingTime: Date.now(),
          });
          if (result.point) addPoint(result.point);
          addLog(`STATUS:POS:${nextAngle.toFixed(1)}`, 'in');
          addLog(`{"angle":${nextAngle.toFixed(1)},"dist":${result.distance.toFixed(2)},"z":0}`, 'in');
          return { ...prev, currentAngle: nextAngle };
        });
      } else if (upper.startsWith('RIGHT') || upper.startsWith('TR')) {
        let deg = turntable.stepSize;
        const parts = upper.split(':');
        if (parts[1]) deg = parseFloat(parts[1]) || turntable.stepSize;

        setTurntable((prev) => {
          const nextAngle = Math.min(settings.fullSweepAngle, prev.currentAngle + deg);
          const result = simulateUltrasonicPing(nextAngle, 0, simulatedModel, settings);
          setSensorData({
            distanceCm: result.distance,
            rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
            valid: true,
            lastPingTime: Date.now(),
          });
          if (result.point) addPoint(result.point);
          addLog(`STATUS:POS:${nextAngle.toFixed(1)}`, 'in');
          addLog(`{"angle":${nextAngle.toFixed(1)},"dist":${result.distance.toFixed(2)},"z":0}`, 'in');
          return { ...prev, currentAngle: nextAngle };
        });
      } else if (upper.startsWith('GOTO')) {
        const parts = upper.split(':');
        const target = parseFloat(parts[1] || '0');
        const nextAngle = Math.max(0, Math.min(settings.fullSweepAngle, target));
        setTurntable((prev) => {
          const result = simulateUltrasonicPing(nextAngle, 0, simulatedModel, settings);
          setSensorData({
            distanceCm: result.distance,
            rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
            valid: true,
            lastPingTime: Date.now(),
          });
          if (result.point) addPoint(result.point);
          addLog(`STATUS:POS:${nextAngle.toFixed(1)}`, 'in');
          addLog(`{"angle":${nextAngle.toFixed(1)},"dist":${result.distance.toFixed(2)},"z":0}`, 'in');
          return { ...prev, currentAngle: nextAngle };
        });
      } else if (upper === 'HOME') {
        setTurntable((prev) => ({ ...prev, currentAngle: 0 }));
        addLog('STATUS:HOMED (0°)', 'in');
      } else if (upper === 'PING') {
        const result = simulateUltrasonicPing(turntable.currentAngle, 0, simulatedModel, settings);
        setSensorData({
          distanceCm: result.distance,
          rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
          valid: true,
          lastPingTime: Date.now(),
        });
        if (result.point) addPoint(result.point);
        addLog(`{"angle":${turntable.currentAngle.toFixed(1)},"dist":${result.distance.toFixed(2)},"z":0}`, 'in');
      } else if (upper.startsWith('SCAN:START') || upper === 'SCAN') {
        startAutomatedScan();
      } else if (upper === 'SCAN:STOP' || upper === 'STOP') {
        stopAutomatedScan();
      }
    },
    [turntable.stepSize, turntable.currentAngle, simulatedModel, settings, addPoint, addLog]
  );

  // Start Automated 3D scan
  const startAutomatedScan = useCallback(() => {
    setScanStatus('scanning');
    addLog('Iniciando escaneo 3D automático...', 'sys');

    if (connectionMode === 'serial') {
      sendSerialCommand(`SCAN:${settings.angularResolution}`);
    } else if (connectionMode === 'simulator') {
      if (scanIntervalRef.current) clearInterval(scanIntervalRef.current);

      let currentDeg = 0;
      let currentZLayer = 0;
      const totalLayers = settings.verticalLayers || 1;
      const step = settings.angularResolution || 2;
      const sweepLimit = settings.fullSweepAngle || 360;

      scanIntervalRef.current = setInterval(() => {
        const zHeight = currentZLayer * 1.5; // cm height per slice
        const result = simulateUltrasonicPing(currentDeg, zHeight, simulatedModel, settings);

        setTurntable((prev) => ({ ...prev, currentAngle: currentDeg }));
        setSensorData({
          distanceCm: result.distance,
          rawEchoMicros: Math.round(result.distance / 0.0343 * 2),
          valid: true,
          lastPingTime: Date.now(),
        });

        if (result.point) {
          addPoint(result.point);
        }

        addLog(`{"angle":${currentDeg},"dist":${result.distance.toFixed(2)},"z":${zHeight.toFixed(1)}}`, 'in');

        currentDeg += step;

        if (currentDeg > sweepLimit) {
          currentZLayer++;
          if (currentZLayer < totalLayers) {
            currentDeg = 0;
            addLog(`Capa vertical ${currentZLayer + 1}/${totalLayers} iniciada`, 'sys');
          } else {
            // Completed scan
            clearInterval(scanIntervalRef.current);
            scanIntervalRef.current = null;
            setScanStatus('completed');
            addLog('STATUS:SCAN_COMPLETE - Nube de puntos 3D generada', 'sys');
          }
        }
      }, 45); // ~22 points per second
    }
  }, [connectionMode, settings, simulatedModel, sendSerialCommand, addPoint, addLog]);

  // Stop scan
  const stopAutomatedScan = useCallback(() => {
    if (scanIntervalRef.current) {
      clearInterval(scanIntervalRef.current);
      scanIntervalRef.current = null;
    }
    setScanStatus('idle');
    addLog('Escaneo 3D detenido', 'sys');

    if (connectionMode === 'serial') {
      sendSerialCommand('STOP');
    }
  }, [connectionMode, sendSerialCommand, addLog]);

  // Turn left handler
  const handleTurnLeft = useCallback(
    (degrees: number) => {
      sendSerialCommand(`LEFT:${degrees}`);
    },
    [sendSerialCommand]
  );

  // Turn right handler
  const handleTurnRight = useCallback(
    (degrees: number) => {
      sendSerialCommand(`RIGHT:${degrees}`);
    },
    [sendSerialCommand]
  );

  // Goto angle handler
  const handleGotoAngle = useCallback(
    (angle: number) => {
      sendSerialCommand(`GOTO:${angle}`);
    },
    [sendSerialCommand]
  );

  // Home handler
  const handleHome = useCallback(() => {
    sendSerialCommand('HOME');
  }, [sendSerialCommand]);

  // Step size change
  const handleStepChange = useCallback((step: number) => {
    setTurntable((prev) => ({ ...prev, stepSize: step }));
  }, []);

  // Ping sensor handler
  const handlePingSensor = useCallback(() => {
    sendSerialCommand('PING');
  }, [sendSerialCommand]);

  // Clear points
  const handleClearPoints = useCallback(() => {
    setPoints([]);
    addLog('Nube de puntos reiniciada', 'sys');
  }, [addLog]);

  // Connect toggle
  const handleConnect = useCallback(() => {
    if (connectionMode === 'serial') {
      connectSerial();
    } else {
      connectSimulator();
    }
  }, [connectionMode, connectSerial, connectSimulator]);

  const handleDisconnect = useCallback(() => {
    if (connectionMode === 'serial') {
      disconnectSerial();
    } else {
      disconnectSimulator();
    }
  }, [connectionMode, disconnectSerial, disconnectSimulator]);

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      if (scanIntervalRef.current) {
        clearInterval(scanIntervalRef.current);
      }
    };
  }, []);

  return {
    connectionMode,
    setConnectionMode,
    isConnected,
    isConnecting,
    portName,
    baudRate,
    setBaudRate,
    simulatedModel,
    setSimulatedModel,
    scanStatus,
    turntable,
    sensorData,
    points,
    settings,
    setSettings,
    serialLogs,
    handleConnect,
    handleDisconnect,
    handleTurnLeft,
    handleTurnRight,
    handleGotoAngle,
    handleHome,
    handleStepChange,
    handlePingSensor,
    handleClearPoints,
    startAutomatedScan,
    stopAutomatedScan,
    sendSerialCommand,
    clearSerialLogs: () => setSerialLogs([]),
  };
}
