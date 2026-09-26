import React from 'react';
import { RoverSweepState } from '../../types/worldDiscoverer';
import { Timer, Compass, ArrowRightLeft, ShieldAlert, Radio, CheckCircle2 } from 'lucide-react';

interface SweepProgressTimerProps {
  roverState: RoverSweepState;
}

export const SweepProgressTimer: React.FC<SweepProgressTimerProps> = ({ roverState }) => {
  const {
    currentAngle,
    relativeAngle,
    sweepDirection,
    isScanning,
    scanMode,
    isObstacleDetected,
    obstacleDistanceCm,
    obstacleThresholdCm,
    sweepPeriodSeconds,
    elapsedInSweepSeconds,
    totalSweepsCompleted,
    currentDistanceCm,
  } = roverState;

  const isNarrow = scanMode === 'narrow_patrol';
  const isClose = currentDistanceCm <= obstacleThresholdCm;

  return (
    <div
      className={`rounded-xl p-3.5 flex flex-col gap-3 transition-colors border shadow-lg ${
        isClose
          ? 'bg-rose-950/40 border-rose-500/50 shadow-rose-950/30'
          : 'bg-neutral-900/90 border-neutral-800'
      }`}
    >
      {/* Header with Mode Status */}
      <div className="flex items-center justify-between text-xs">
        <div className="flex items-center gap-2">
          {isClose ? (
            <ShieldAlert className="w-4 h-4 text-rose-400 animate-pulse" />
          ) : (
            <Radio className="w-4 h-4 text-emerald-400" />
          )}
          <span className="font-semibold text-neutral-100">
            {isNarrow
              ? 'Sondeo Frontal Estrecho (±15°)'
              : 'Sondeo Panorámico 180° (-90° a +90°)'}
          </span>
        </div>

        <span
          className={`px-2 py-0.5 rounded text-[11px] font-mono font-medium border ${
            isClose
              ? 'bg-rose-500/20 text-rose-300 border-rose-500/40 animate-pulse'
              : 'bg-emerald-500/15 text-emerald-300 border-emerald-500/30'
          }`}
        >
          {isClose ? '¡OBSTÁCULO <= 40cm!' : 'VÍA DESPEJADA'}
        </span>
      </div>

      {/* Prominent Distance Return in CM (Direct response to "Devuelve los cm al que esta el objeto") */}
      <div
        className={`p-3 rounded-lg border flex items-center justify-between font-mono ${
          isClose
            ? 'bg-rose-950/60 border-rose-500/40 text-rose-100'
            : 'bg-neutral-950 border-neutral-800 text-neutral-100'
        }`}
      >
        <div className="flex flex-col">
          <span className="text-[10px] tracking-wider text-neutral-400 uppercase font-sans font-medium">
            Distancia al Objeto Detectado
          </span>
          <div className="flex items-baseline gap-1.5 mt-0.5">
            <span
              className={`text-2xl font-bold tabular-nums tracking-tight ${
                isClose
                  ? 'text-rose-400'
                  : currentDistanceCm < 100
                  ? 'text-amber-400'
                  : 'text-emerald-400'
              }`}
            >
              {currentDistanceCm.toFixed(1)}
            </span>
            <span className="text-sm font-semibold text-neutral-400">cm</span>
          </div>
        </div>

        {/* Status indicator tag */}
        <div className="flex flex-col items-end text-[11px]">
          <span className="text-neutral-400 text-[10px]">UMBRAL DISPARO</span>
          <span className="font-bold text-neutral-300">{obstacleThresholdCm} cm</span>
          {isClose ? (
            <span className="text-rose-400 font-bold text-[10px] flex items-center gap-1 mt-0.5">
              <span>●</span> Barrido 180° Activado
            </span>
          ) : (
            <span className="text-emerald-400 font-medium text-[10px] flex items-center gap-1 mt-0.5">
              <CheckCircle2 className="w-3 h-3" /> Vigilancia 0° ± 15°
            </span>
          )}
        </div>
      </div>

      {/* Angle Gauge: Absolute Servo (0°-180°) & Relative to Center 90° (-90° to +90°) */}
      <div className="grid grid-cols-2 gap-2 text-xs bg-neutral-950/70 p-2.5 rounded-lg border border-neutral-800/80 font-mono">
        <div className="flex items-center gap-2">
          <Compass className="w-3.5 h-3.5 text-cyan-400" />
          <div className="flex flex-col">
            <span className="text-[10px] text-neutral-500 font-sans">ÁNGULO SERVO</span>
            <span className="text-cyan-300 font-bold text-sm">
              {Math.round(currentAngle)}°
              <span className="text-[11px] text-neutral-500 font-normal ml-1">
                (Inicio: 90°)
              </span>
            </span>
          </div>
        </div>

        <div className="flex items-center gap-2 border-l border-neutral-800 pl-2.5">
          <ArrowRightLeft className="w-3.5 h-3.5 text-amber-400" />
          <div className="flex flex-col">
            <span className="text-[10px] text-neutral-500 font-sans">RELATIVO AL FRENTE</span>
            <span
              className={`font-bold text-sm ${
                relativeAngle > 0
                  ? 'text-emerald-400'
                  : relativeAngle < 0
                  ? 'text-amber-400'
                  : 'text-neutral-200'
              }`}
            >
              {relativeAngle > 0 ? `+${relativeAngle}° Izq` : relativeAngle < 0 ? `${relativeAngle}° Der` : '0° Centro'}
            </span>
          </div>
        </div>
      </div>

      {/* Visual Scan Arc representation */}
      <div className="flex flex-col gap-1 text-[11px] text-neutral-400">
        <div className="flex items-center justify-between text-[10px] font-mono">
          <span className="text-neutral-500">
            {isNarrow ? 'Rango Activo: 75° a 105° (0° ±15°)' : 'Rango Activo: 0° a 180° (-90° a +90°)'}
          </span>
          <span className="text-neutral-500">
            {sweepDirection === 'forward' ? 'Hacia Izquierda (+)' : 'Hacia Derecha (-)'}
          </span>
        </div>

        {/* Arc Progress Bar */}
        <div className="relative w-full h-2 bg-neutral-950 rounded-full overflow-hidden border border-neutral-800">
          <div
            className={`h-full transition-all duration-75 ease-linear rounded-full ${
              isClose
                ? 'bg-gradient-to-r from-rose-500 to-amber-500 animate-pulse'
                : 'bg-gradient-to-r from-emerald-500 via-teal-400 to-cyan-400'
            }`}
            style={{
              width: `${Math.min(100, Math.max(0, (currentAngle / 180) * 100))}%`,
            }}
          />
        </div>
      </div>
    </div>
  );
};
