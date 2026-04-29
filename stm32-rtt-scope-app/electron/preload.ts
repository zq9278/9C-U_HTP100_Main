import { contextBridge, ipcRenderer } from 'electron';
import { ScopeConfig } from './types';

contextBridge.exposeInMainWorld('rttScope', {
  getConfig: () => ipcRenderer.invoke('app:get-config'),
  saveConfig: (config: ScopeConfig) => ipcRenderer.invoke('app:save-config', config),
  loadConfigFile: () => ipcRenderer.invoke('app:load-config-file'),
  saveConfigFileAs: (config: ScopeConfig) => ipcRenderer.invoke('app:save-config-file-as', config),
  startRtt: (exe?: string) => ipcRenderer.invoke('rtt:start', exe),
  stopRtt: () => ipcRenderer.invoke('rtt:stop'),
  sendCommand: (command: string) => ipcRenderer.invoke('rtt:send', command),
  exportCsv: (csv: string) => ipcRenderer.invoke('csv:export', csv),
  onStatus: (callback: (payload: { running: boolean; message: string }) => void) => {
    const listener = (_event: Electron.IpcRendererEvent, payload: { running: boolean; message: string }) => callback(payload);
    ipcRenderer.on('rtt-status', listener);
    return () => ipcRenderer.removeListener('rtt-status', listener);
  },
  onLog: (callback: (payload: { ts: number; channel: number; line: string }) => void) => {
    const listener = (_event: Electron.IpcRendererEvent, payload: { ts: number; channel: number; line: string }) => callback(payload);
    ipcRenderer.on('rtt-log', listener);
    return () => ipcRenderer.removeListener('rtt-log', listener);
  },
  onVariables: (callback: (payload: unknown) => void) => {
    const listener = (_event: Electron.IpcRendererEvent, payload: unknown) => callback(payload);
    ipcRenderer.on('rtt-variables', listener);
    return () => ipcRenderer.removeListener('rtt-variables', listener);
  }
});
