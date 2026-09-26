import React from 'react';
import { BotPose, TrajectoryPoint, Waypoint } from '../../types/worldDiscoverer';
import {
  ArrowUp,
  ArrowDown,
  ArrowLeft,
  ArrowRight,
  Square,
  Bot,
  MapPin,
  RotateCcw,
  Sparkles,
  Compass,
  Footprints,
  Activity,
} from 'lucide-react';

interface BotNavigationCardProps {
  botPose: BotPose;
  trajectory: TrajectoryPoint[];
  waypoints: Waypoint[];
  isAutonomous: boolean;
  onToggleAutonomous: () => void;
  onDriveForward: () => void;
  onDriveBackward: () => void;
  onTurnLeft: () => void;
  onTurnRight: () => void;
  onStopBot: () => void;
  onAddWaypoint: () => void;
  onResetRoute: () => void;
  disabled?: boolean;
}

export const BotNavigationCard: React.FC<BotNavigationCardProps> = ({
  botPose,
  trajectory,
  waypoints,
  isAutonomous,
  onToggleAutonomous,
  onDriveForward,
  onDriveBackward,
  onTurnLeft,
  onTurnRight,
  onStopBot,
  onAddWaypoint,
  onResetRoute,
  disabled = false,
}) => {
  return (
    <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-4 flex flex-col gap-4 shadow-lg">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <Bot className="w-4 h-4 text-cyan-400" />
          <h3 className="text-sm font-semibold text-neutral-100">
            Conducción & Registro de Trayecto
          </h3>
        </div>
        <span
          className={`text-[11px] font-mono px-2 py-0.5 rounded-full border ${
            isAutonomous
              ? 'bg-purple-500/20 text-purple-300 border-purple-500/40 animate-pulse'
              : botPose.isMoving
              ? 'bg-emerald-500/20 text-emerald-300 border-emerald-500/40'
              : 'bg-neutral-800 text-neutral-400 border-neutral-700'
          }`}
        >
          {isAutonomous ? 'Navegación Autónoma' : botPose.isMoving ? 'En Movimiento' : 'Manual / Listo'}
        </span>
      </div>

      {/* Trajectory Metrics Quick Bar */}
      <div className="grid grid-cols-3 gap-2 bg-neutral-950 p-2.5 rounded-lg border border-neutral-800/80 text-center font-mono">
        <div>
          <span className="text-[10px] text-neutral-500 block">DISTANCIA</span>
          <span className="text-emerald-400 font-bold text-sm">
            {(botPose.totalDistanceCm / 100).toFixed(2)} m
          </span>
        </div>
        <div>
          <span className="text-[10px] text-neutral-500 block">RUMBO / POS</span>
          <span className="text-cyan-400 font-bold text-sm">
            {Math.round(botPose.heading)}°
          </span>
        </div>
        <div>
          <span className="text-[10px] text-neutral-500 block">PUNTOS RUTA</span>
          <span className="text-amber-400 font-bold text-sm">
            {trajectory.length}
          </span>
        </div>
      </div>

      {/* Autonomous Navigation Toggle Button */}
      <button
        onClick={onToggleAutonomous}
        disabled={disabled}
        className={`w-full flex items-center justify-center gap-2 px-3 py-2.5 rounded-lg text-xs font-semibold transition-all cursor-pointer ${
          isAutonomous
            ? 'bg-purple-600 hover:bg-purple-500 text-white shadow-lg shadow-purple-500/20'
            : 'bg-neutral-800 hover:bg-neutral-750 text-neutral-200 border border-neutral-700/80'
        }`}
      >
        <Sparkles className="w-4 h-4 text-purple-400" />
        <span>
          {isAutonomous ? 'Detener Exploración Autónoma' : 'Activar Exploración Autónoma (Auto-Mapeo)'}
        </span>
      </button>

      {/* D-Pad Virtual Steering Controls */}
      <div className="flex flex-col items-center gap-1.5 py-1">
        {/* Forward */}
        <button
          onClick={onDriveForward}
          disabled={disabled || isAutonomous}
          className="w-12 h-11 bg-neutral-800 hover:bg-neutral-700 active:bg-cyan-600 text-neutral-200 active:text-white rounded-lg flex items-center justify-center transition-colors border border-neutral-700/80 cursor-pointer disabled:opacity-40"
          title="Avanzar (W / Flecha Arriba)"
        >
          <ArrowUp className="w-5 h-5" />
        </button>

        {/* Left - Stop - Right */}
        <div className="flex items-center gap-2">
          <button
            onClick={onTurnLeft}
            disabled={disabled || isAutonomous}
            className="w-12 h-11 bg-neutral-800 hover:bg-neutral-700 active:bg-cyan-600 text-neutral-200 active:text-white rounded-lg flex items-center justify-center transition-colors border border-neutral-700/80 cursor-pointer disabled:opacity-40"
            title="Girar Izquierda (A / Flecha Izquierda)"
          >
            <ArrowLeft className="w-5 h-5" />
          </button>

          <button
            onClick={onStopBot}
            disabled={disabled}
            className="w-12 h-11 bg-rose-500/20 hover:bg-rose-500/30 active:bg-rose-600 text-rose-400 active:text-white rounded-lg flex items-center justify-center transition-colors border border-rose-500/40 cursor-pointer"
            title="Detener Bot (Espacio)"
          >
            <Square className="w-4 h-4 fill-current" />
          </button>

          <button
            onClick={onTurnRight}
            disabled={disabled || isAutonomous}
            className="w-12 h-11 bg-neutral-800 hover:bg-neutral-700 active:bg-cyan-600 text-neutral-200 active:text-white rounded-lg flex items-center justify-center transition-colors border border-neutral-700/80 cursor-pointer disabled:opacity-40"
            title="Girar Derecha (D / Flecha Derecha)"
          >
            <ArrowRight className="w-5 h-5" />
          </button>
        </div>

        {/* Backward */}
        <button
          onClick={onDriveBackward}
          disabled={disabled || isAutonomous}
          className="w-12 h-11 bg-neutral-800 hover:bg-neutral-700 active:bg-cyan-600 text-neutral-200 active:text-white rounded-lg flex items-center justify-center transition-colors border border-neutral-700/80 cursor-pointer disabled:opacity-40"
          title="Retroceder (S / Flecha Abajo)"
        >
          <ArrowDown className="w-5 h-5" />
        </button>
      </div>

      {/* Keyboard Shortcut Hint */}
      <div className="text-[11px] text-neutral-400 bg-neutral-950/60 p-2 rounded-lg border border-neutral-800/60 text-center font-mono">
        <span>Teclas: </span>
        <span className="text-neutral-200 font-semibold">W, A, S, D</span>
        <span className="text-neutral-500"> o </span>
        <span className="text-neutral-200 font-semibold">Flechas</span>
        <span className="text-neutral-500"> · </span>
        <span className="text-neutral-200 font-semibold">Espacio</span> (Parar)
      </div>

      {/* Route Actions (Add Waypoint & Reset Route) */}
      <div className="grid grid-cols-2 gap-2 pt-1 border-t border-neutral-800/80">
        <button
          onClick={onAddWaypoint}
          disabled={disabled}
          className="flex items-center justify-center gap-1.5 px-3 py-2 rounded-lg text-xs font-medium text-cyan-300 bg-cyan-950/40 border border-cyan-500/30 hover:bg-cyan-900/50 transition-colors cursor-pointer"
        >
          <MapPin className="w-3.5 h-3.5" />
          <span>Fijar Hito / Hito #{waypoints.length + 1}</span>
        </button>

        <button
          onClick={onResetRoute}
          disabled={disabled}
          className="flex items-center justify-center gap-1.5 px-3 py-2 rounded-lg text-xs font-medium text-neutral-400 bg-neutral-800 hover:text-neutral-100 hover:bg-neutral-750 transition-colors cursor-pointer"
        >
          <RotateCcw className="w-3.5 h-3.5" />
          <span>Reiniciar Ruta</span>
        </button>
      </div>
    </div>
  );
};
