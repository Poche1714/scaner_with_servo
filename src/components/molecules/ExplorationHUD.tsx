import React from 'react';
import { ExplorationStats, GameVisualTheme } from '../../types/worldDiscoverer';
import { Volume2, VolumeX, Eye, Footprints, ShieldAlert, Sparkles, Compass } from 'lucide-react';

interface ExplorationHUDProps {
  stats: ExplorationStats;
  theme: GameVisualTheme;
  onThemeChange: (theme: GameVisualTheme) => void;
  isMuted: boolean;
  onToggleMute: () => void;
}

export const ExplorationHUD: React.FC<ExplorationHUDProps> = ({
  stats,
  theme,
  onThemeChange,
  isMuted,
  onToggleMute,
}) => {
  return (
    <div className="grid grid-cols-2 sm:grid-cols-3 lg:grid-cols-5 gap-3">
      {/* 1. Bot Trajectory & Traveled Distance (Directly answers the user's route request) */}
      <div className="bg-neutral-900/90 border border-emerald-500/30 rounded-xl p-3 flex flex-col justify-between shadow-sm">
        <div className="flex items-center justify-between text-xs text-neutral-400">
          <span className="flex items-center gap-1.5 font-medium text-emerald-400">
            <Footprints className="w-3.5 h-3.5" />
            <span>Trayecto del Bot</span>
          </span>
          <span className="text-[11px] text-emerald-400 font-mono">Ruta</span>
        </div>
        <div className="mt-2 flex items-baseline gap-1.5">
          <span className="text-2xl font-bold font-mono text-neutral-100 tabular-nums">
            {stats.totalDistanceTraveledM.toFixed(2)}
          </span>
          <span className="text-xs text-neutral-500">metros</span>
        </div>
        <div className="flex items-center justify-between text-[11px] text-neutral-400 mt-2 font-mono">
          <span>{stats.routePointsCount} hitos/puntos</span>
          <span className="text-emerald-400">Activo</span>
        </div>
      </div>

      {/* 2. Discovered Area & Fog Cleared */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3 flex flex-col justify-between">
        <div className="flex items-center justify-between text-xs text-neutral-400">
          <span className="flex items-center gap-1.5">
            <Eye className="w-3.5 h-3.5 text-cyan-400" />
            <span>Mundo Revelado</span>
          </span>
          <span className="text-[11px] text-cyan-400 font-mono">{stats.totalAreaM2} m²</span>
        </div>
        <div className="mt-2 flex items-baseline gap-1.5">
          <span className="text-2xl font-bold font-mono text-neutral-100 tabular-nums">
            {stats.fogClearedPercentage}%
          </span>
          <span className="text-xs text-neutral-500">niebla despejada</span>
        </div>
        <div className="w-full bg-neutral-950 rounded-full h-1.5 mt-2 overflow-hidden">
          <div
            className="bg-cyan-500 h-full rounded-full transition-all duration-300"
            style={{ width: `${stats.fogClearedPercentage}%` }}
          />
        </div>
      </div>

      {/* 3. Obstacles Detected */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3 flex flex-col justify-between">
        <div className="flex items-center justify-between text-xs text-neutral-400">
          <span className="flex items-center gap-1.5">
            <ShieldAlert className="w-3.5 h-3.5 text-amber-400" />
            <span>Obstáculos Mapeados</span>
          </span>
        </div>
        <div className="mt-2 flex items-baseline gap-1.5">
          <span className="text-2xl font-bold font-mono text-neutral-100 tabular-nums">
            {stats.obstaclesDetectedCount}
          </span>
          <span className="text-xs text-neutral-500">puntos 2D</span>
        </div>
        <span className="text-[11px] text-neutral-500 mt-2">Muros y límites registrados</span>
      </div>

      {/* 4. Anomalies / POIs */}
      <div className="bg-neutral-900/90 border border-neutral-800 rounded-xl p-3 flex flex-col justify-between">
        <div className="flex items-center justify-between text-xs text-neutral-400">
          <span className="flex items-center gap-1.5">
            <Sparkles className="w-3.5 h-3.5 text-purple-400" />
            <span>Sector Actual</span>
          </span>
        </div>
        <div className="mt-2 flex items-baseline gap-1.5">
          <span className="text-sm font-semibold text-neutral-200 truncate" title={stats.currentSectorName}>
            {stats.currentSectorName}
          </span>
        </div>
        <span className="text-[11px] text-purple-400/80 mt-2 truncate">
          Rango máx: {stats.maxRangeReachedCm} cm
        </span>
      </div>

      {/* 5. Theme & Audio Controls */}
      <div className="col-span-2 sm:col-span-3 lg:col-span-1 bg-neutral-900/90 border border-neutral-800 rounded-xl p-3 flex flex-col justify-between gap-2">
        <div className="flex items-center justify-between">
          <span className="text-xs text-neutral-400">Estilo Visual:</span>
          <button
            onClick={onToggleMute}
            className={`p-1.5 rounded-lg text-xs transition-colors cursor-pointer ${
              isMuted
                ? 'bg-neutral-800 text-neutral-500'
                : 'bg-emerald-500/20 text-emerald-300 border border-emerald-500/40'
            }`}
            title={isMuted ? 'Activar Sonar Sonoro' : 'Silenciar Sonar'}
          >
            {isMuted ? <VolumeX className="w-3.5 h-3.5" /> : <Volume2 className="w-3.5 h-3.5" />}
          </button>
        </div>

        <select
          value={theme}
          onChange={(e) => onThemeChange(e.target.value as GameVisualTheme)}
          className="w-full bg-neutral-950 text-neutral-200 border border-neutral-800 rounded-lg text-xs py-1.5 px-2 focus:outline-none focus:border-emerald-500"
        >
          <option value="tactical_radar">Radar Táctico (Verde)</option>
          <option value="dungeon_pixel">Mazmorra RPG (Ámbar)</option>
          <option value="cyber_sonar">Ciber Sonar (Violeta)</option>
          <option value="blueprint">Plano Técnico (Azul)</option>
        </select>
      </div>
    </div>
  );
};
