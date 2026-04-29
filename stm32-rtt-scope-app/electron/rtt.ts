import { ChildProcessWithoutNullStreams, spawn } from 'child_process';
import { EventEmitter } from 'events';
import { ParsedVariables, VariableMeta } from './types';

export interface RttEvents {
  log: [{ ts: number; channel: number; line: string }];
  variables: [ParsedVariables];
  status: [{ running: boolean; message: string }];
}

export class RttController extends EventEmitter {
  private proc?: ChildProcessWithoutNullStreams;
  private stdoutBuffer = '';

  start(exe: string) {
    if (this.proc) {
      return;
    }

    this.emitStatus(true, `Starting ${exe}`);
    try {
      this.proc = spawn(exe, [], { shell: false });
    } catch (error) {
      this.proc = undefined;
      this.emitStatus(false, `Failed to start RTT client: ${String(error)}`);
      return;
    }

    this.proc.stdout.on('data', chunk => this.onStdout(chunk.toString('utf8')));
    this.proc.stderr.on('data', chunk => {
      const text = stripAnsi(chunk.toString('utf8')).trim();
      if (text) {
        this.emit('log', { ts: Date.now(), channel: -1, line: text });
      }
    });
    this.proc.on('error', error => {
      this.emit('log', { ts: Date.now(), channel: -1, line: `RTT error: ${error.message}` });
      this.emitStatus(false, error.message);
    });
    this.proc.on('exit', code => {
      this.proc = undefined;
      this.emitStatus(false, `RTT client exited: ${code ?? 'unknown'}`);
    });
  }

  stop() {
    this.proc?.kill();
    this.proc = undefined;
    this.emitStatus(false, 'Stopped');
  }

  isRunning() {
    return Boolean(this.proc);
  }

  write(command: string) {
    const line = command.trim();
    if (!line || !this.proc?.stdin.writable) {
      return false;
    }
    this.proc.stdin.write(`${line}\n`);
    this.emit('log', { ts: Date.now(), channel: 0, line: `> ${line}` });
    return true;
  }

  override on<K extends keyof RttEvents>(eventName: K, listener: (...args: RttEvents[K]) => void): this {
    return super.on(eventName, listener);
  }

  override emit<K extends keyof RttEvents>(eventName: K, ...args: RttEvents[K]): boolean {
    return super.emit(eventName, ...args);
  }

  private onStdout(text: string) {
    this.stdoutBuffer += text;
    const lines = this.stdoutBuffer.split(/\r?\n/);
    this.stdoutBuffer = lines.pop() ?? '';
    for (const raw of lines) {
      const line = stripAnsi(raw).trim();
      if (!line) {
        continue;
      }

      const variables = parseVariableLine(line);
      if (variables) {
        this.emit('variables', variables);
      } else {
        this.emit('log', { ts: Date.now(), channel: 0, line });
      }
    }
  }

  private emitStatus(running: boolean, message: string) {
    this.emit('status', { running, message });
  }
}

export function parseVariableLine(line: string): ParsedVariables | undefined {
  const trimmed = line.trim();
  if (!trimmed.startsWith('$VAR')) {
    return undefined;
  }

  const frame = trimmed.replace(/^\$VAR,?/, '');
  const values: ParsedVariables['values'] = {};
  const tokens = frame.split(/[,\t ]+/).filter(Boolean);
  for (const token of tokens) {
    const match = token.match(/^([A-Za-z_][\w./:-]*)=(.+)$/);
    if (!match) {
      continue;
    }

    const raw = match[2].replace(/^"|"$/g, '');
    const numericValue = Number(raw);
    const numeric = raw !== '' && Number.isFinite(numericValue);
    values[match[1]] = { value: numeric ? numericValue : raw, numeric };
  }

  return Object.keys(values).length ? { ts: Date.now(), values } : undefined;
}

export function inferVariableMeta(key: string): VariableMeta {
  const lower = key.toLowerCase();
  if (lower.includes('temp')) {
    return { key, unit: '\u2103', group: 'temperature' };
  }
  if (lower.includes('pressure') || lower.includes('press')) {
    return { key, unit: 'kPa', group: 'pressure' };
  }
  if (lower.includes('pwm') || lower.includes('duty')) {
    return { key, unit: '%', group: 'control' };
  }
  if (lower.includes('volt')) {
    return { key, unit: 'V', group: 'power' };
  }
  if (lower.includes('curr')) {
    return { key, unit: 'A', group: 'power' };
  }
  if (lower.includes('pid') || lower.includes('kp') || lower.includes('ki') || lower.includes('kd') || lower.includes('error') || lower.includes('integral') || lower.includes('derivative')) {
    return { key, unit: '', group: 'pid' };
  }
  return { key, unit: '', group: 'general' };
}

function stripAnsi(text: string) {
  return text.replace(/\x1B\[[0-?]*[ -/]*[@-~]/g, '');
}
