// Logging utility for ChatKit integration
interface LogEntry {
  timestamp: string;
  level: 'info' | 'warn' | 'error' | 'debug';
  message: string;
  context?: Record<string, any>;
}

class ChatKitLogger {
  private static instance: ChatKitLogger;
  private logs: LogEntry[] = [];
  private maxLogs: number = 1000; // Keep last 1000 logs
  private enabled: boolean = true;

  private constructor() {}

  public static getInstance(): ChatKitLogger {
    if (!ChatKitLogger.instance) {
      ChatKitLogger.instance = new ChatKitLogger();
    }
    return ChatKitLogger.instance;
  }

  setEnabled(enabled: boolean): void {
    this.enabled = enabled;
  }

  info(message: string, context?: Record<string, any>): void {
    if (this.enabled) {
      this.log('info', message, context);
    }
  }

  warn(message: string, context?: Record<string, any>): void {
    if (this.enabled) {
      this.log('warn', message, context);
    }
  }

  error(message: string, context?: Record<string, any>): void {
    if (this.enabled) {
      this.log('error', message, context);
    }
  }

  debug(message: string, context?: Record<string, any>): void {
    if (this.enabled) {
      this.log('debug', message, context);
    }
  }

  private log(level: LogEntry['level'], message: string, context?: Record<string, any>): void {
    const logEntry: LogEntry = {
      timestamp: new Date().toISOString(),
      level,
      message,
      context
    };

    // Add to logs array
    this.logs.push(logEntry);

    // Maintain max log count
    if (this.logs.length > this.maxLogs) {
      this.logs = this.logs.slice(-this.maxLogs);
    }

    // Also log to console
    switch (level) {
      case 'info':
        console.info(`[ChatKit ${level.toUpperCase()}]`, message, context || '');
        break;
      case 'warn':
        console.warn(`[ChatKit ${level.toUpperCase()}]`, message, context || '');
        break;
      case 'error':
        console.error(`[ChatKit ${level.toUpperCase()}]`, message, context || '');
        break;
      case 'debug':
        console.debug(`[ChatKit ${level.toUpperCase()}]`, message, context || '');
        break;
    }
  }

  getLogs(): LogEntry[] {
    return [...this.logs]; // Return a copy
  }

  clearLogs(): void {
    this.logs = [];
  }

  exportLogs(): string {
    return JSON.stringify(this.logs, null, 2);
  }
}

// Create global logger instance
const chatKitLogger = ChatKitLogger.getInstance();
export default chatKitLogger;