import { CSSProperties, useEffect, useMemo, useRef, useState } from 'react';
import * as echarts from 'echarts';
import { Activity, Download, FolderOpen, Gauge, Moon, Pause, Play, RotateCcw, Save, Send, Settings, Square, Sun, Trash2, X } from 'lucide-react';
import { ChartConfig, ChartType, LogEntry, ScopeConfig, VariableMeta, VariableSample } from './types';

const MAX_POINTS = 500;
const COLORS = ['#35e8ff', '#64ffb2', '#ffd166', '#ff7a90', '#9a8cff', '#5ad1ff'];
const ACCENTS = ['#35e8ff', '#64ffb2', '#ffd166', '#ff5370', '#9a8cff', '#4d8dff'];

const emptyConfig: ScopeConfig = {
  variables: {},
  charts: [],
  pid: { name: 'pid_heat', kp: 1.2, ki: 0.08, kd: 0.01 },
  settings: {
    rttClientPath: 'JLinkRTTClient.exe',
    projectConfigPath: '',
    autoStart: false,
    themeMode: 'dark',
    accentColor: '#35e8ff',
    backgroundColor: '#071014',
    panelColor: '#0d1a20',
    textColor: '#d7f8ff',
    fontSize: 13,
    sidebarWidth: 300,
    inspectorWidth: 370,
    pidPanelHeight: 260
  }
};

type Buffers = Record<string, Array<[number, number]>>;
type Latest = Record<string, number | string>;

export default function App() {
  const [config, setConfig] = useState<ScopeConfig>(emptyConfig);
  const [variables, setVariables] = useState<Record<string, VariableMeta>>({});
  const [latest, setLatest] = useState<Latest>({});
  const [buffers, setBuffers] = useState<Buffers>({});
  const [charts, setCharts] = useState<ChartConfig[]>([]);
  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [running, setRunning] = useState(false);
  const [paused, setPaused] = useState(false);
  const [filter, setFilter] = useState('');
  const [sampleRate, setSampleRate] = useState(0);
  const [csvRows, setCsvRows] = useState<Array<Record<string, number | string>>>([]);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const samplesThisSecond = useRef(0);
  const appStyle = useMemo(() => ({
    '--sidebar-width': `${config.settings.sidebarWidth || 300}px`,
    '--inspector-width': `${config.settings.inspectorWidth || 370}px`,
    '--pid-panel-height': `${config.settings.pidPanelHeight || 260}px`,
    '--accent': config.settings.accentColor || '#35e8ff',
    '--bg': config.settings.backgroundColor || (config.settings.themeMode === 'light' ? '#f5f7f9' : '#071014'),
    '--panel': config.settings.panelColor || (config.settings.themeMode === 'light' ? '#ffffff' : '#0d1a20'),
    '--text': config.settings.textColor || (config.settings.themeMode === 'light' ? '#17262d' : '#d7f8ff'),
    '--app-font-size': `${config.settings.fontSize || 13}px`
  }) as CSSProperties, [
    config.settings.sidebarWidth,
    config.settings.inspectorWidth,
    config.settings.pidPanelHeight,
    config.settings.accentColor,
    config.settings.backgroundColor,
    config.settings.panelColor,
    config.settings.textColor,
    config.settings.fontSize,
    config.settings.themeMode
  ]);

  useEffect(() => {
    window.rttScope.getConfig().then(next => {
      setConfig(next);
      setVariables(next.variables || {});
      setCharts(next.charts || []);
    });

    const offStatus = window.rttScope.onStatus(status => {
      setRunning(status.running);
      appendLog({ ts: Date.now(), channel: -1, line: status.message });
    });
    const offLog = window.rttScope.onLog(appendLog);
    const offVariables = window.rttScope.onVariables(payload => ingestVariables(payload.samples, payload.variables));
    const timer = window.setInterval(() => {
      setSampleRate(samplesThisSecond.current);
      samplesThisSecond.current = 0;
    }, 1000);

    return () => {
      offStatus();
      offLog();
      offVariables();
      window.clearInterval(timer);
    };
  }, [paused]);

  function appendLog(entry: LogEntry) {
    setLogs(prev => [...prev.slice(-699), entry]);
  }

  function ingestVariables(samples: VariableSample[], nextVariables: Record<string, VariableMeta>) {
    setVariables(prev => ({ ...prev, ...nextVariables }));
    samplesThisSecond.current += samples.length;
    if (paused) {
      return;
    }

    const nextLatest: Latest = {};
    const nextCsv: Array<Record<string, number | string>> = [];
    setBuffers(prev => {
      const next: Buffers = { ...prev };
      for (const sample of samples) {
        const row: Record<string, number | string> = { ts: sample.ts };
        for (const [key, entry] of Object.entries(sample.values)) {
          nextLatest[key] = entry.value;
          row[key] = entry.value;
          if (entry.numeric) {
            const buffer = next[key] ? [...next[key]] : [];
            buffer.push([sample.ts, Number(entry.value)]);
            next[key] = buffer.slice(-MAX_POINTS);
          }
        }
        nextCsv.push(row);
      }
      return next;
    });
    setLatest(prev => ({ ...prev, ...nextLatest }));
    setCsvRows(prev => [...prev, ...nextCsv].slice(-20000));
  }

  const groupedVariables = useMemo(() => {
    const groups = new Map<string, VariableMeta[]>();
    Object.values(variables)
      .filter(item => !filter || item.key.toLowerCase().includes(filter.toLowerCase()) || (item.group || '').toLowerCase().includes(filter.toLowerCase()))
      .sort((a, b) => `${a.group}.${a.key}`.localeCompare(`${b.group}.${b.key}`))
      .forEach(item => {
        const group = item.group || 'general';
        groups.set(group, [...(groups.get(group) || []), item]);
      });
    return groups;
  }, [variables, filter]);

  async function start() {
    await window.rttScope.startRtt(config.settings.rttClientPath);
  }

  async function stop() {
    await window.rttScope.stopRtt();
  }

  function addChart(keys: string[], type: ChartType = 'line') {
    const chart: ChartConfig = {
      id: `chart_${Date.now()}_${Math.random().toString(16).slice(2)}`,
      title: keys.join(' + '),
      type,
      keys,
      x: 0,
      y: charts.length,
      w: keys.length > 1 ? 6 : 4,
      h: type === 'number' ? 2 : 3
    };
    setCharts(prev => [...prev, chart]);
  }

  function updateChart(id: string, patch: Partial<ChartConfig>) {
    setCharts(prev => prev.map(chart => (chart.id === id ? { ...chart, ...patch } : chart)));
  }

  function removeChart(id: string) {
    setCharts(prev => prev.filter(chart => chart.id !== id));
  }

  function moveChart(sourceId: string, targetId: string) {
    if (sourceId === targetId) {
      return;
    }
    setCharts(prev => {
      const from = prev.findIndex(chart => chart.id === sourceId);
      const to = prev.findIndex(chart => chart.id === targetId);
      if (from < 0 || to < 0) {
        return prev;
      }
      const next = [...prev];
      const [item] = next.splice(from, 1);
      next.splice(to, 0, item);
      return next.map((chart, index) => ({ ...chart, y: index }));
    });
  }

  function removeVariable(key: string) {
    const nextVariables = { ...variables };
    delete nextVariables[key];
    const nextCharts = charts
      .map(chart => {
        const keys = chart.keys.filter(item => item !== key);
        return { ...chart, keys, title: keys.join(' + ') };
      })
      .filter(chart => chart.keys.length > 0);

    setVariables(nextVariables);
    setLatest(prev => {
      const next = { ...prev };
      delete next[key];
      return next;
    });
    setBuffers(prev => {
      const next = { ...prev };
      delete next[key];
      return next;
    });
    setCharts(nextCharts);

    const nextConfig = { ...config, variables: nextVariables, charts: nextCharts };
    setConfig(nextConfig);
    window.rttScope.saveConfig(nextConfig);
  }

  function clearVariables() {
    setVariables({});
    setLatest({});
    setBuffers({});
    setCharts([]);
    setCsvRows([]);
    const nextConfig = { ...config, variables: {}, charts: [] };
    setConfig(nextConfig);
    window.rttScope.saveConfig(nextConfig);
  }

  function saveConfig() {
    const next = { ...config, variables, charts };
    setConfig(next);
    window.rttScope.saveConfig(next);
  }

  function updateSettings(patch: Partial<ScopeConfig['settings']>) {
    setConfig(prev => ({ ...prev, settings: { ...prev.settings, ...patch } }));
  }

  function applyThemeMode(mode: 'dark' | 'light') {
    updateSettings({
      themeMode: mode,
      backgroundColor: mode === 'light' ? '#f5f7f9' : '#071014',
      panelColor: mode === 'light' ? '#ffffff' : '#0d1a20',
      textColor: mode === 'light' ? '#17262d' : '#d7f8ff'
    });
  }

  function resetLayout() {
    updateSettings({ sidebarWidth: 300, inspectorWidth: 370, pidPanelHeight: 260, fontSize: 13 });
  }

  function startResize(side: 'left' | 'right', event: React.PointerEvent<HTMLDivElement>) {
    const startX = event.clientX;
    const startLeft = config.settings.sidebarWidth || 300;
    const startRight = config.settings.inspectorWidth || 370;

    const onMove = (moveEvent: PointerEvent) => {
      if (side === 'left') {
        updateSettings({ sidebarWidth: clamp(startLeft + moveEvent.clientX - startX, 220, 520) });
      } else {
        updateSettings({ inspectorWidth: clamp(startRight - moveEvent.clientX + startX, 280, 620) });
      }
    };
    const onUp = () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
  }

  function startInspectorSplitResize(event: React.PointerEvent<HTMLDivElement>) {
    const startY = event.clientY;
    const startHeight = config.settings.pidPanelHeight || 260;
    const onMove = (moveEvent: PointerEvent) => {
      updateSettings({ pidPanelHeight: clamp(startHeight + moveEvent.clientY - startY, 190, 460) });
    };
    const onUp = () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
  }

  async function saveAs() {
    const next = await window.rttScope.saveConfigFileAs({ ...config, variables, charts });
    setConfig(next);
  }

  async function loadFile() {
    const next = await window.rttScope.loadConfigFile();
    setConfig(next);
    setVariables(next.variables || {});
    setCharts(next.charts || []);
  }

  function exportCsv() {
    const keys = Array.from(new Set(csvRows.flatMap(row => Object.keys(row).filter(key => key !== 'ts')))).sort();
    const rows = [['timestamp', ...keys], ...csvRows.map(row => [new Date(Number(row.ts)).toISOString(), ...keys.map(key => row[key] ?? '')])];
    const csv = rows.map(row => row.map(cell => `"${String(cell).replace(/"/g, '""')}"`).join(',')).join('\n');
    window.rttScope.exportCsv(csv);
  }

  function sendPid(verb: 'GET' | 'SET' | 'SAVE') {
    const pid = config.pid;
    const command = verb === 'GET'
      ? `GET,${pid.name}`
      : `${verb},${pid.name},kp=${pid.kp.toFixed(2)},ki=${pid.ki.toFixed(3)},kd=${pid.kd.toFixed(3)}`;
    window.rttScope.sendCommand(command);
  }

  return (
    <div className={`app ${config.settings.themeMode === 'light' ? 'lightTheme' : 'darkTheme'}`} style={appStyle}>
      <aside className="sidebar">
        <div className="brand">
          <div className="brandMark"><Activity size={22} /></div>
          <div>
            <h1>STM32 RTT Scope</h1>
            <span className={running ? 'online' : 'offline'}>{running ? 'Connected' : 'Disconnected'} | {sampleRate} Hz</span>
          </div>
        </div>

        <div className="pathRow">
          <input
            value={config.settings.rttClientPath}
            onChange={event => updateSettings({ rttClientPath: event.target.value })}
          />
        </div>

        <div className="toolbar">
          <button onClick={start}><Play size={15} />Start</button>
          <button onClick={stop}><Square size={15} />Stop</button>
          <button onClick={() => setPaused(value => !value)}>{paused ? <Play size={15} /> : <Pause size={15} />}{paused ? 'Resume' : 'Pause'}</button>
        </div>

        <div className="sectionHeader">
          <div className="sectionTitle">Variables</div>
          <button className="miniTextBtn" onClick={clearVariables}>Clear</button>
        </div>
        <input className="search" placeholder="Filter variables" value={filter} onChange={event => setFilter(event.target.value)} />
        <div className="variableList">
          {[...groupedVariables.entries()].map(([group, items]) => (
            <div key={group}>
              <div className="groupTitle">{group}</div>
              {items.map(meta => (
                <div
                  className={`variable ${isAlarm(meta, latest[meta.key]) ? 'alarm' : ''}`}
                  key={meta.key}
                  draggable
                  onDragStart={event => event.dataTransfer.setData('text/plain', meta.key)}
                >
                  <div>
                    <strong>{meta.key}</strong>
                    <span>{meta.unit || '-'}</span>
                  </div>
                  <b>{formatValue(latest[meta.key])}</b>
                  <button
                    className="variableDelete"
                    title="Delete variable"
                    onClick={event => {
                      event.stopPropagation();
                      removeVariable(meta.key);
                    }}
                  >
                    <X size={13} />
                  </button>
                </div>
              ))}
            </div>
          ))}
        </div>
      </aside>

      <div className="resizeHandle leftHandle" onPointerDown={event => startResize('left', event)} />

      <main className="workspace">
        <header className="topbar">
          <div className="canvasTitle"><Gauge size={18} />Realtime Canvas</div>
          <div className="toolbar">
            <button onClick={() => addChart(Object.keys(variables).filter(key => buffers[key]).slice(0, 4), 'line')}>Combo</button>
            <button onClick={loadFile}><FolderOpen size={15} />Load</button>
            <button onClick={saveConfig}><Save size={15} />Save</button>
            <button onClick={saveAs}>Save As</button>
            <button onClick={exportCsv}><Download size={15} />CSV</button>
            <button onClick={() => setSettingsOpen(true)}><Settings size={15} />Settings</button>
          </div>
        </header>

        <section
          className={`canvas ${charts.length ? '' : 'empty'}`}
          onDragOver={event => event.preventDefault()}
          onDrop={event => {
            if ((event.target as HTMLElement).closest('.chartCard')) {
              return;
            }
            const chartId = event.dataTransfer.getData('application/x-chart-id');
            if (chartId) {
              setCharts(prev => [...prev.filter(chart => chart.id !== chartId), ...prev.filter(chart => chart.id === chartId)]);
              return;
            }
            const key = event.dataTransfer.getData('text/plain');
            if (key) {
              addChart([key]);
            }
          }}
        >
          {charts.map(chart => (
            <ChartCard
              key={chart.id}
              chart={chart}
              variables={variables}
              latest={latest}
              buffers={buffers}
              onUpdate={patch => updateChart(chart.id, patch)}
              onRemove={() => removeChart(chart.id)}
              onMoveBefore={sourceId => moveChart(sourceId, chart.id)}
            />
          ))}
        </section>
      </main>

      <div className="resizeHandle rightHandle" onPointerDown={event => startResize('right', event)} />

      <aside className="inspector">
        <section className="panel pidPanel">
          <div className="sectionTitle">PID Parameters</div>
          <label>Target<input value={config.pid.name} onChange={event => setConfig({ ...config, pid: { ...config.pid, name: event.target.value } })} /></label>
          <PidInput label="Kp" value={config.pid.kp} max={20} step={0.01} onChange={kp => setConfig({ ...config, pid: { ...config.pid, kp } })} />
          <PidInput label="Ki" value={config.pid.ki} max={5} step={0.001} onChange={ki => setConfig({ ...config, pid: { ...config.pid, ki } })} />
          <PidInput label="Kd" value={config.pid.kd} max={5} step={0.001} onChange={kd => setConfig({ ...config, pid: { ...config.pid, kd } })} />
          <div className="toolbar">
            <button onClick={() => sendPid('GET')}>GET</button>
            <button onClick={() => sendPid('SET')}><Send size={15} />SET</button>
            <button onClick={() => sendPid('SAVE')}>SAVE</button>
          </div>
        </section>

        <div className="resizeHandleY" onPointerDown={startInspectorSplitResize} />

        <section className="panel logPanel">
          <div className="sectionTitle">RTT Log</div>
          <div className="logView">
            {logs.map((log, index) => (
              <div className="logLine" key={`${log.ts}-${index}`}>
                [{new Date(log.ts).toLocaleTimeString()} {log.channel >= 0 ? `CH${log.channel}` : 'SYS'}] {log.line}
              </div>
            ))}
          </div>
        </section>
      </aside>

      {settingsOpen && (
        <SettingsPanel
          config={config}
          onClose={() => setSettingsOpen(false)}
          onSettingsChange={updateSettings}
          onThemeMode={applyThemeMode}
          onResetLayout={resetLayout}
          onSave={saveConfig}
        />
      )}
    </div>
  );
}

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(max, value));
}

function ChartCard(props: {
  chart: ChartConfig;
  variables: Record<string, VariableMeta>;
  latest: Latest;
  buffers: Buffers;
  onUpdate: (patch: Partial<ChartConfig>) => void;
  onRemove: () => void;
  onMoveBefore: (sourceId: string) => void;
}) {
  const { chart, variables, latest, buffers, onUpdate, onRemove, onMoveBefore } = props;
  const ref = useRef<HTMLDivElement>(null);
  const instance = useRef<echarts.ECharts | null>(null);

  useEffect(() => {
    if (!ref.current) {
      return;
    }
    instance.current = echarts.init(ref.current, null, { renderer: 'canvas' });
    const resize = () => instance.current?.resize();
    window.addEventListener('resize', resize);
    return () => {
      window.removeEventListener('resize', resize);
      instance.current?.dispose();
    };
  }, []);

  useEffect(() => {
    if (!instance.current) {
      return;
    }
    instance.current.setOption(buildChartOption(chart, variables, latest, buffers), true);
  }, [chart, variables, latest, buffers]);

  function startChartResize(event: React.PointerEvent<HTMLButtonElement>) {
    event.preventDefault();
    event.stopPropagation();
    const startX = event.clientX;
    const startY = event.clientY;
    const startW = chart.w;
    const startH = chart.h;
    const onMove = (moveEvent: PointerEvent) => {
      onUpdate({
        w: clamp(startW + Math.round((moveEvent.clientX - startX) / 74), 2, 12),
        h: clamp(startH + Math.round((moveEvent.clientY - startY) / 92), 2, 8)
      });
    };
    const onUp = () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
    };

    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
  }

  return (
    <article
      className={`chartCard ${chart.keys.some(key => isAlarm(variables[key], latest[key])) ? 'alarm' : ''}`}
      style={{ gridColumn: `span ${chart.w}`, gridRow: `span ${chart.h}` }}
      draggable
      onDragStart={event => {
        event.dataTransfer.setData('application/x-chart-id', chart.id);
        event.dataTransfer.effectAllowed = 'move';
      }}
      onDragOver={event => event.preventDefault()}
      onDrop={event => {
        const sourceId = event.dataTransfer.getData('application/x-chart-id');
        if (sourceId) {
          onMoveBefore(sourceId);
          return;
        }
        const key = event.dataTransfer.getData('text/plain');
        if (key && !chart.keys.includes(key)) {
          onUpdate({ keys: [...chart.keys, key], title: [...chart.keys, key].join(' + ') });
        }
      }}
    >
      <div className="cardHead">
        <strong>{chart.title}</strong>
        <div className="cardTools">
          <select value={chart.type} onChange={event => onUpdate({ type: event.target.value as ChartType })}>
            <option value="line">line</option>
            <option value="bar">bar</option>
            <option value="gauge">gauge</option>
            <option value="pie">pie</option>
            <option value="number">number</option>
          </select>
          <button onClick={() => onUpdate({ w: Math.max(2, chart.w - 1) })}>-</button>
          <button onClick={() => onUpdate({ w: Math.min(12, chart.w + 1) })}>+</button>
          <button onClick={() => onUpdate({ h: Math.min(7, chart.h + 1) })}>H</button>
          <button onClick={onRemove}><Trash2 size={14} /></button>
        </div>
      </div>
      <div className="chartSurface" ref={ref} />
      <button className="chartResizeGrip" title="Drag to resize" onPointerDown={startChartResize} />
    </article>
  );
}

function SettingsPanel(props: {
  config: ScopeConfig;
  onClose: () => void;
  onSettingsChange: (patch: Partial<ScopeConfig['settings']>) => void;
  onThemeMode: (mode: 'dark' | 'light') => void;
  onResetLayout: () => void;
  onSave: () => void;
}) {
  const { config, onClose, onSettingsChange, onThemeMode, onResetLayout, onSave } = props;
  const settings = config.settings;

  return (
    <div className="settingsOverlay">
      <section className="settingsDialog">
        <header className="settingsHead">
          <div>
            <h2>Settings</h2>
            <span>Theme, colors, font, and layout are saved with the scope config.</span>
          </div>
          <button className="iconButton" onClick={onClose}><X size={17} /></button>
        </header>

        <div className="settingsBody">
          <div className="settingsGroup">
            <div className="sectionTitle">Theme</div>
            <div className="segmented">
              <button className={settings.themeMode === 'dark' ? 'active' : ''} onClick={() => onThemeMode('dark')}><Moon size={15} />Dark</button>
              <button className={settings.themeMode === 'light' ? 'active' : ''} onClick={() => onThemeMode('light')}><Sun size={15} />Light</button>
            </div>
            <label>Font size
              <input type="range" min={11} max={18} step={1} value={settings.fontSize || 13} onChange={event => onSettingsChange({ fontSize: Number(event.target.value) })} />
              <input type="number" min={11} max={18} value={settings.fontSize || 13} onChange={event => onSettingsChange({ fontSize: Number(event.target.value) })} />
            </label>
          </div>

          <div className="settingsGroup">
            <div className="sectionTitle">Colors</div>
            <ColorField label="Accent" value={settings.accentColor || '#35e8ff'} onChange={accentColor => onSettingsChange({ accentColor })} />
            <ColorField label="Background" value={settings.backgroundColor || '#071014'} onChange={backgroundColor => onSettingsChange({ backgroundColor })} />
            <ColorField label="Panel" value={settings.panelColor || '#0d1a20'} onChange={panelColor => onSettingsChange({ panelColor })} />
            <ColorField label="Text" value={settings.textColor || '#d7f8ff'} onChange={textColor => onSettingsChange({ textColor })} />
            <div className="swatches">
              {ACCENTS.map(color => (
                <button className="swatch" key={color} style={{ backgroundColor: color }} title={color} onClick={() => onSettingsChange({ accentColor: color })} />
              ))}
            </div>
          </div>

          <div className="settingsGroup">
            <div className="sectionTitle">Layout</div>
            <NumberRange label="Variable panel" value={settings.sidebarWidth || 300} min={220} max={520} onChange={sidebarWidth => onSettingsChange({ sidebarWidth })} />
            <NumberRange label="Right panel" value={settings.inspectorWidth || 370} min={280} max={620} onChange={inspectorWidth => onSettingsChange({ inspectorWidth })} />
            <NumberRange label="PID panel height" value={settings.pidPanelHeight || 260} min={190} max={460} onChange={pidPanelHeight => onSettingsChange({ pidPanelHeight })} />
            <button onClick={onResetLayout}><RotateCcw size={15} />Reset layout</button>
          </div>

          <div className="settingsGroup">
            <div className="sectionTitle">RTT</div>
            <label>JLinkRTTClient
              <input value={settings.rttClientPath} onChange={event => onSettingsChange({ rttClientPath: event.target.value })} />
            </label>
          </div>
        </div>

        <footer className="settingsFoot">
          <button onClick={onClose}>Close</button>
          <button onClick={() => { onSave(); onClose(); }}><Save size={15} />Save config</button>
        </footer>
      </section>
    </div>
  );
}

function ColorField(props: { label: string; value: string; onChange: (value: string) => void }) {
  return (
    <label className="colorField">
      {props.label}
      <input type="color" value={props.value} onChange={event => props.onChange(event.target.value)} />
      <input value={props.value} onChange={event => props.onChange(event.target.value)} />
    </label>
  );
}

function NumberRange(props: { label: string; value: number; min: number; max: number; onChange: (value: number) => void }) {
  return (
    <label>
      {props.label}
      <input type="range" min={props.min} max={props.max} value={props.value} onChange={event => props.onChange(Number(event.target.value))} />
      <input type="number" min={props.min} max={props.max} value={props.value} onChange={event => props.onChange(Number(event.target.value))} />
    </label>
  );
}

function PidInput(props: { label: string; value: number; max: number; step: number; onChange: (value: number) => void }) {
  return (
    <label>
      {props.label}
      <input type="number" value={props.value} step={props.step} onChange={event => props.onChange(Number(event.target.value))} />
      <input type="range" min={0} max={props.max} step={props.step} value={props.value} onChange={event => props.onChange(Number(event.target.value))} />
    </label>
  );
}

function buildChartOption(chart: ChartConfig, variables: Record<string, VariableMeta>, latest: Latest, buffers: Buffers): echarts.EChartsOption {
  const unit = chart.keys.length === 1 ? variables[chart.keys[0]]?.unit || '' : '';
  const textColor = getCssVar('--text', '#d7f8ff');
  const mutedColor = getCssVar('--muted', '#7ca0a8');
  const lineColor = getCssVar('--line', '#1d3c46');
  const logBg = getCssVar('--log-bg', '#071014');
  const common = {
    animation: false,
    backgroundColor: 'transparent',
    color: COLORS,
    textStyle: { color: textColor }
  };

  if (chart.type === 'number') {
    const key = chart.keys[0];
    return {
      ...common,
      graphic: [{ type: 'text', left: 'center', top: 'middle', style: { text: `${formatValue(latest[key])}${unit ? ' ' + unit : ''}`, fill: getCssVar('--green', '#64ffb2'), fontSize: 34, fontWeight: 700 } }]
    };
  }

  if (chart.type === 'gauge') {
    const key = chart.keys[0];
    const value = Number(latest[key] ?? 0);
    return {
      ...common,
      series: [{ type: 'gauge', min: 0, max: Math.max(100, value * 1.2), progress: { show: true }, detail: { formatter: `{value}${unit}`, color: textColor }, data: [{ value, name: key }] }]
    };
  }

  if (chart.type === 'pie') {
    return {
      ...common,
      tooltip: { trigger: 'item' },
      series: [{ type: 'pie', radius: ['42%', '72%'], data: chart.keys.map(key => ({ name: key, value: Math.abs(Number(latest[key] ?? 0)) })) }]
    };
  }

  const cartesianType = chart.type === 'bar' ? 'bar' : 'line';
  return {
    ...common,
    tooltip: { trigger: 'axis', backgroundColor: logBg, borderColor: lineColor, textStyle: { color: textColor } },
    legend: { top: 0, textStyle: { color: mutedColor } },
    grid: { left: 42, right: 16, top: 28, bottom: 30 },
    xAxis: {
      type: 'time',
      axisLine: { lineStyle: { color: lineColor } },
      axisLabel: {
        color: mutedColor,
        formatter: (value: number) => {
          const date = new Date(value);
          return `${date.getHours().toString().padStart(2, '0')}:${date.getMinutes().toString().padStart(2, '0')}:${date.getSeconds().toString().padStart(2, '0')}.${Math.floor(date.getMilliseconds() / 100)}`;
        }
      }
    },
    yAxis: { type: 'value', name: unit, splitLine: { lineStyle: { color: lineColor } }, axisLabel: { color: mutedColor } },
    series: chart.keys.map(key => ({
      name: `${key}${variables[key]?.unit ? ` (${variables[key].unit})` : ''}`,
      type: cartesianType,
      showSymbol: false,
      smooth: cartesianType === 'line' ? 0.18 : false,
      lineStyle: { width: 2 },
      barMaxWidth: 14,
      data: (buffers[key] || []).map(([ts, value]) => [ts, value])
    }))
  };
}

function getCssVar(name: string, fallback: string) {
  const root = document.querySelector('.app');
  if (!root) {
    return fallback;
  }
  return getComputedStyle(root).getPropertyValue(name).trim() || fallback;
}

function isAlarm(meta: VariableMeta | undefined, value: number | string | undefined) {
  const numeric = Number(value);
  if (!meta || !Number.isFinite(numeric)) {
    return false;
  }
  return (Number.isFinite(meta.warnMin) && numeric < Number(meta.warnMin)) || (Number.isFinite(meta.warnMax) && numeric > Number(meta.warnMax));
}

function formatValue(value: number | string | undefined) {
  if (typeof value === 'number') {
    return Number.isInteger(value) ? String(value) : value.toFixed(2);
  }
  return value === undefined ? '--' : String(value);
}
