export type ChartType = 'line' | 'bar' | 'gauge' | 'pie' | 'number';

export interface AppSettings {
  rttClientPath: string;
  projectConfigPath: string;
  autoStart: boolean;
  themeMode: 'dark' | 'light';
  accentColor: string;
  backgroundColor: string;
  panelColor: string;
  textColor: string;
  fontSize: number;
  sidebarWidth: number;
  inspectorWidth: number;
  pidPanelHeight: number;
}

export interface VariableMeta {
  key: string;
  unit?: string;
  group?: string;
  warnMin?: number;
  warnMax?: number;
}

export interface ChartConfig {
  id: string;
  title: string;
  type: ChartType;
  keys: string[];
  x: number;
  y: number;
  w: number;
  h: number;
}

export interface PidConfig {
  name: string;
  kp: number;
  ki: number;
  kd: number;
}

export interface ScopeConfig {
  variables: Record<string, VariableMeta>;
  charts: ChartConfig[];
  pid: PidConfig;
  settings: AppSettings;
}

export interface VariableSample {
  ts: number;
  values: Record<string, { value: number | string; numeric: boolean }>;
}

export interface RttStatus {
  running: boolean;
  message: string;
}

export interface LogEntry {
  ts: number;
  channel: number;
  line: string;
}

export interface RttScopeApi {
  getConfig(): Promise<ScopeConfig>;
  saveConfig(config: ScopeConfig): Promise<ScopeConfig>;
  loadConfigFile(): Promise<ScopeConfig>;
  saveConfigFileAs(config: ScopeConfig): Promise<ScopeConfig>;
  startRtt(exe?: string): Promise<boolean>;
  stopRtt(): Promise<boolean>;
  sendCommand(command: string): Promise<boolean>;
  exportCsv(csv: string): Promise<boolean>;
  onStatus(callback: (payload: RttStatus) => void): () => void;
  onLog(callback: (payload: LogEntry) => void): () => void;
  onVariables(callback: (payload: { samples: VariableSample[]; variables: Record<string, VariableMeta> }) => void): () => void;
}

declare global {
  interface Window {
    rttScope: RttScopeApi;
  }
}
