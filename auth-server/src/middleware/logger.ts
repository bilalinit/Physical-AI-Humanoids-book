import winston from 'winston';
import { Request, Response, NextFunction } from 'express';

// Create logger instance
const logger = winston.createLogger({
  level: process.env.LOG_LEVEL || 'info',
  format: winston.format.combine(
    winston.format.timestamp(),
    winston.format.errors({ stack: true }),
    winston.format.splat(),
    winston.format.json()
  ),
  defaultMeta: { service: 'auth-server' },
  transports: [
    // Write to all logs with level `info` and below to `combined.log`
    new winston.transports.File({ filename: 'logs/combined.log' }),
    // Write all logs error (and below) to `error.log`
    new winston.transports.File({ filename: 'logs/error.log', level: 'error' })
  ]
});

// If we're not in production, log to the `console` with the format:
if (process.env.NODE_ENV !== 'production') {
  logger.add(new winston.transports.Console({
    format: winston.format.combine(
      winston.format.colorize(),
      winston.format.simple()
    )
  }));
}

// Logging middleware
const requestLogger = (req: Request, res: Response, next: NextFunction) => {
  const start = Date.now();

  res.on('finish', () => {
    const duration = Date.now() - start;
    logger.info(`${req.method} ${req.originalUrl}`, {
      status: res.statusCode,
      duration: `${duration}ms`,
      ip: req.ip,
      userAgent: req.get('User-Agent'),
      userId: (req as any).user ? (req as any).user.id : null
    });
  });

  next();
};

// Error logging middleware
const errorLogger = (err: any, req: Request, res: Response, next: NextFunction) => {
  logger.error(`${req.method} ${req.originalUrl}`, {
    error: err.message,
    stack: err.stack,
    ip: req.ip,
    userAgent: req.get('User-Agent'),
    userId: (req as any).user ? (req as any).user.id : null
  });

  next(err);
};

// Specific logging for user registration operations
const logUserRegistration = (user: any, operation: string) => {
  logger.info('User registration operation', {
    operation,
    userId: user.id,
    email: user.email,
    name: user.name,
    timestamp: new Date().toISOString(),
    educationLevel: user.educationLevel,
    programmingExperience: user.programmingExperience,
    roboticsBackground: user.roboticsBackground
  });
};

// Specific logging for authentication operations
const logAuthentication = (user: any, operation: string, success: boolean = true, details: any = {}) => {
  logger.info('Authentication operation', {
    operation,
    success,
    userId: user ? user.id : null,
    email: user ? user.email : null,
    timestamp: new Date().toISOString(),
    ...details
  });
};

export {
  logger,
  requestLogger,
  errorLogger,
  logUserRegistration,
  logAuthentication
};