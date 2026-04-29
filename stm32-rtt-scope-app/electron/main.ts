import { app, BrowserWindow, dialog, ipcMain } from 'electron';
import * as fs from 'fs';
import * as path from 'path';
import { RttController, inferVariableMeta } from './rtt';
import { ScopeConfig, defaultConfig } from './types';

const rtt = new RttController();
let mainWindow: BrowserWindow | undefined;
let pendingSamples: unknown[] = [];
let flushTimer: NodeJS.Timeout | undefined;
let currentConfig = normalizeConfig();

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1500,
    height: 920,
    minWidth: 1100,
    minHeight: 720,
    backgroundColor: '#071014',
    title: 'STM32 RTT Scope',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false
    }
  });

  if (process.env.VITE_DEV_SERVER_URL) {
    mainWindow.loadURL(process.env.VITE_DEV_SERVER_URL);
    mainWindow.webContents.openDevTools({ mode: 'detach' });
  } else {
    mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'));
  }

  mainWindow.on('closed', () => {
    mainWindow = undefined;
  });
}

app.whenReady().then(() => {
  currentConfig = loadLocalConfig();
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  rtt.stop();
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

rtt.on('log', entry => {
  send('rtt-log', entry);
});

rtt.on('status', status => {
  send('rtt-status', status);
});

rtt.on('variables', sample => {
  for (const key of Object.keys(sample.values)) {
    if (!currentConfig.variables[key]) {
      currentConfig.variables[key] = inferVariableMeta(key);
    }
  }

  pendingSamples.push(sample);
  if (pendingSamples.length > 3000) {
    pendingSamples.splice(0, pendingSamples.length - 3000);
  }

  if (!flushTimer) {
    // Batch high-frequency RTT frames so Chromium paints at a steady cadence.
    flushTimer = setTimeout(() => {
      const batch = pendingSamples.splice(0);
      flushTimer = undefined;
      send('rtt-variables', { samples: batch, variables: currentConfig.variables });
    }, 33);
  }
});

ipcMain.handle('app:get-config', () => currentConfig);

ipcMain.handle('app:save-config', async (_event, config: ScopeConfig) => {
  currentConfig = normalizeConfig(config);
  saveLocalConfig(currentConfig);
  if (currentConfig.settings.projectConfigPath) {
    await fs.promises.mkdir(path.dirname(currentConfig.settings.projectConfigPath), { recursive: true });
    await fs.promises.writeFile(currentConfig.settings.projectConfigPath, `${JSON.stringify(currentConfig, null, 2)}\n`, 'utf8');
  }
  return currentConfig;
});

ipcMain.handle('app:load-config-file', async () => {
  const result = await dialog.showOpenDialog(mainWindow!, {
    filters: [{ name: 'STM32 RTT Scope Config', extensions: ['json'] }],
    properties: ['openFile']
  });
  if (result.canceled || !result.filePaths[0]) {
    return currentConfig;
  }
  const file = result.filePaths[0];
  currentConfig = normalizeConfig(JSON.parse(await fs.promises.readFile(file, 'utf8')));
  currentConfig.settings.projectConfigPath = file;
  saveLocalConfig(currentConfig);
  return currentConfig;
});

ipcMain.handle('app:save-config-file-as', async (_event, config: ScopeConfig) => {
  const result = await dialog.showSaveDialog(mainWindow!, {
    defaultPath: config.settings.projectConfigPath || path.join(app.getPath('documents'), 'stm32-rtt-scope.json'),
    filters: [{ name: 'JSON', extensions: ['json'] }]
  });
  if (result.canceled || !result.filePath) {
    return normalizeConfig(config);
  }
  const next = normalizeConfig(config);
  next.settings.projectConfigPath = result.filePath;
  await fs.promises.writeFile(result.filePath, `${JSON.stringify(next, null, 2)}\n`, 'utf8');
  currentConfig = next;
  saveLocalConfig(currentConfig);
  return currentConfig;
});

ipcMain.handle('rtt:start', (_event, exe?: string) => {
  const rttClientPath = exe?.trim() || currentConfig.settings.rttClientPath || 'JLinkRTTClient.exe';
  currentConfig.settings.rttClientPath = rttClientPath;
  saveLocalConfig(currentConfig);
  rtt.start(rttClientPath);
  return rtt.isRunning();
});

ipcMain.handle('rtt:stop', () => {
  rtt.stop();
  return false;
});

ipcMain.handle('rtt:send', (_event, command: string) => rtt.write(command));

ipcMain.handle('csv:export', async (_event, csv: string) => {
  const result = await dialog.showSaveDialog(mainWindow!, {
    defaultPath: path.join(app.getPath('documents'), `stm32-rtt-scope-${new Date().toISOString().replace(/[:.]/g, '-')}.csv`),
    filters: [{ name: 'CSV', extensions: ['csv'] }]
  });
  if (result.canceled || !result.filePath) {
    return false;
  }
  await fs.promises.writeFile(result.filePath, csv, 'utf8');
  return true;
});

function send(channel: string, payload: unknown) {
  mainWindow?.webContents.send(channel, payload);
}

function normalizeConfig(input?: Partial<ScopeConfig>): ScopeConfig {
  return {
    variables: input?.variables && typeof input.variables === 'object' ? input.variables : {},
    charts: Array.isArray(input?.charts) ? input.charts : [],
    pid: {
      name: input?.pid?.name || defaultConfig.pid.name,
      kp: Number(input?.pid?.kp ?? defaultConfig.pid.kp),
      ki: Number(input?.pid?.ki ?? defaultConfig.pid.ki),
      kd: Number(input?.pid?.kd ?? defaultConfig.pid.kd)
    },
    settings: {
      ...defaultConfig.settings,
      ...(input?.settings || {})
    }
  };
}

function getLocalConfigPath() {
  return path.join(app.getPath('userData'), 'stm32-rtt-scope.json');
}

function loadLocalConfig() {
  const file = getLocalConfigPath();
  if (!fs.existsSync(file)) {
    return normalizeConfig();
  }

  try {
    return normalizeConfig(JSON.parse(fs.readFileSync(file, 'utf8')));
  } catch {
    return normalizeConfig();
  }
}

function saveLocalConfig(config: ScopeConfig) {
  fs.mkdirSync(app.getPath('userData'), { recursive: true });
  fs.writeFileSync(getLocalConfigPath(), `${JSON.stringify(config, null, 2)}\n`, 'utf8');
}
