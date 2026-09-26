export interface DiscoveredPoint2D {
  id: string;
  worldX: number; // cm in global coordinate system
  worldY: number; // cm in global coordinate system
  robotX: number; // position of robot when detected
  robotY: number;
  robotHeading: number; // heading of robot when detected
  sensorAngle: number; // 0° to 180° relative to robot
  distanceCm: number; // 2cm to 400cm
  timestamp: number;
  sweepCycle: number;
  hits: number;
  type: 'wall' | 'obstacle' | 'anomaly';
  clearConfirmations?: number; // Contador de validaciones consecutivas > 70 cm (requiere 3 para borrar)
}

export interface TrajectoryPoint {
  id: string;
  x: number; // cm in global coordinate system
  y: number; // cm in global coordinate system
  heading: number; // degrees
  timestamp: number;
  distanceFromStartCm: number;
  speed: number; // cm/s
  isWaypoint?: boolean;
  waypointLabel?: string;
}

export interface Waypoint {
  id: string;
  x: number;
  y: number;
  label: string;
  timestamp: number;
  color?: string;
  icon?: string;
}

export interface BotPose {
  x: number; // cm
  y: number; // cm
  heading: number; // 0° to 360° (90° is North / +Y, 0° is East / +X)
  speed: number; // cm/s
  isMoving: boolean;
  driveMode: 'manual' | 'autonomous' | 'paused';
  totalDistanceCm: number;
}

export type SweepDirection = 'forward' | 'backward';

export type ScanMode = 
  | 'narrow_patrol'           // Vigilancia normal ±15° (de -15° a +15° relativo, 75° a 105° servo)
  | 'obstacle_focused_survey' // Sondeo enfocado del objeto detectado (+25° más y -10° menos, 3 barridos)
  | 'survey_paused'           // Sondeo detenido tras completar los 3 barridos (se reinicia al girar el bot)
  | 'obstacle_panoramic';     // Barrido de seguridad delimitado a -55° y +55° relativo (35° a 145° servo)

export interface RoverSweepState {
  currentAngle: number; // 0 to 180 (90° is center forward / 0° relativo)
  relativeAngle: number; // -90° to +90° (0° is forward)
  targetAngle: number;
  sweepDirection: SweepDirection;
  isScanning: boolean;
  scanMode: ScanMode;
  isObstacleDetected: boolean;
  obstacleDistanceCm: number | null; // Exact cm to detected obstacle
  obstacleDetectedAngle: number | null; // Exact relative angle where obstacle was spotted
  obstacleThresholdCm: number; // 40 cm trigger
  // Focused object survey window: [detected - 10°, detected + 25°]
  surveyMinAngle: number | null;
  surveyMaxAngle: number | null;
  surveyStepDirection: 1 | -1;
  surveyPassesCount: number; // 0 a 3 barridos del objeto
  sweepPeriodSeconds: number; // seconds for cycle
  elapsedInSweepSeconds: number;
  totalSweepsCompleted: number;
  currentDistanceCm: number;
  maxRangeCm: number; // 100, 200, 300, 400 cm
  motorPwm: number; // PWM value between 175 and 198
}

export type GameVisualTheme = 'tactical_radar' | 'dungeon_pixel' | 'cyber_sonar' | 'blueprint';

export type MapEnvironmentPreset = 'dungeon_chamber' | 'lunar_ruins' | 'room_interior' | 'corridor_maze';

export interface RouteStats {
  totalDistanceMeters: number;
  activeDriveTimeSeconds: number;
  pointsRecordedCount: number;
  waypointsCount: number;
  currentSpeedCmS: number;
  averageSpeedCmS: number;
  obstaclesDetectedCount: number;
  fogClearedPercentage: number;
  currentSectorName: string;
  bounds: { minX: number; maxX: number; minY: number; maxY: number };
}

export interface ExplorationStats {
  fogClearedPercentage: number;
  totalAreaM2: number;
  obstaclesDetectedCount: number;
  activeScanTimeSeconds: number;
  currentSectorName: string;
  maxRangeReachedCm: number;
  anomaliesFound: number;
  totalDistanceTraveledM: number;
  routePointsCount: number;
}

export interface UltrasonicPacket {
  angle: number;
  distanceCm: number;
  valid: boolean;
  timestamp: number;
}

export interface SerialErrorInfo {
  code: 'LOCKED_OR_IN_USE' | 'PERMISSION_DENIED' | 'UNSUPPORTED' | 'DISCONNECTED' | 'UNKNOWN';
  title: string;
  message: string;
  reasons: string[];
  solutions: string[];
}
