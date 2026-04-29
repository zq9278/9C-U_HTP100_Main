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

export interface ParsedVariables {
  ts: number;
  values: Record<string, { value: number | string; numeric: boolean }>;
}

export const defaultConfig: ScopeConfig = {
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
