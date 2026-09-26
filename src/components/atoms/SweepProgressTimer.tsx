import React from 'react';
import { RoverSweepState } from '../../types/worldDiscoverer';
import { Timer, Compass, ArrowRightLeft } from 'lucide-react';

interface SweepProgressTimerProps {
  roverState: RoverSweepState;
}

export const SweepProgressTimer: React.FC<SweepProgressTimerProps> = ({ roverState }) => {
  const {
    currentAngle,
    sweepDirection,
    isScanning,
    sweepPeriodSeconds,
    elapsedInSweepSeconds,
    totalSweepsCompleted,
  } = roverState;

  const percent = Math.min(100, Math.max(0, (elapsedInSweepSeconds / sweepPeriodSeconds) * 100));

  return (
    <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3.5 flex flex-col gap-2.5">
      <div className="flex items-center justify-between text-xs">
        <div className="flex items-center gap-2 text-neutral-300 font-medium">
          <Timer className="w-4 h-4 text-emerald-400" />
          <span>Ciclo de Sondeo (180° en {sweepPeriodSeconds}s)</span>
        </div>
        <div className="flex items-center gap-1.5 text-neutral-400 font-mono text-[11px] tabular-nums">
          <ArrowRightLeft className="w-3.5 h-3.5 text-neutral-500" />
          <span>
            {sweepDirection === 'forward' ? '0° ➔ 180°' : '180° ➔ 0°'}
          </span>
          <span className="text-neutral-600">·</span>
          <span>Barrido #{totalSweepsCompleted + 1}</span>
        </div>
      </div>

      {/* Progress Bar with 20s counter */}
      <div className="relative w-full h-2.5 bg-neutral-950 rounded-full overflow-hidden border border-neutral-800">
        <div
          className={`h-full transition-all duration-100 ease-linear rounded-full ${
            isScanning ? 'bg-gradient-to-r from-emerald-500 via-teal-400 to-cyan-400' : 'bg-neutral-600'
          }`}
          style={{ width: `${percent}%` }}
        />
      </div>

      <div className="flex items-center justify-between text-xs text-neutral-400 font-mono tabular-nums">
        <div className="flex items-center gap-1.5">
          <span className="text-neutral-500">Tiempo:</span>
          <span className="text-neutral-200 font-semibold">{elapsedInSweepSeconds.toFixed(1)}s</span>
          <span className="text-neutral-600">/</span>
          <span>{sweepPeriodSeconds.toFixed(1)}s</span>
        </div>

        <div className="flex items-center gap-1.5">
          <Compass className="w-3.5 h-3.5 text-cyan-400" />
          <span className="text-neutral-500">Ángulo Servo:</span>
          <span className="text-cyan-300 font-semibold text-sm">{Math.round(currentAngle)}°</span>
        </div>
      </div>
    </div>
  );
};
