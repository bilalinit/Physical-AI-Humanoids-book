import dotenv from 'dotenv';
dotenv.config();
import express, { Express, Request, Response, NextFunction } from 'express';
import cookieParser from 'cookie-parser';
import { corsMiddleware } from './middleware/cors.js';
import { requestLogger } from './middleware/logger.js';
import { auth } from './config/auth.js';
import { toNodeHandler } from 'better-auth/node'; // Import toNodeHandler
import routes from './routes/index.js';

const app: Express = express();
const PORT = process.env.PORT || 3001;

// Middleware
app.use(requestLogger); // Logging middleware should be first
app.use(express.json({ limit: '10mb' }));
app.use(express.urlencoded({ extended: true, limit: '10mb' }));
app.use(cookieParser());
app.use(corsMiddleware);

// Mount custom routes (access at /api/auth/signup, /api/auth/signin, etc.)
app.use('/api', routes);

// Mount Better Auth routes (catch-all for other auth methods)
app.all('/api/auth/*', toNodeHandler(auth));

// Error handling middleware
app.use((err: any, req: Request, res: Response, next: NextFunction) => {
  console.error('Unhandled error:', err);
  res.status(500).json({
    error: 'Internal server error',
    details: process.env.NODE_ENV === 'development' ? err.message : undefined
  });
});

// 404 handler
app.use('*', (req: Request, res: Response) => {
  res.status(404).json({
    error: 'Route not found'
  });
});

// Start server
app.listen(PORT, () => {
  console.log(`Auth server running on port ${PORT}`);
  console.log(`Environment: ${process.env.NODE_ENV || 'development'}`);
  console.log(`Base URL: ${process.env.BETTER_AUTH_URL || `http://localhost:${PORT}`}`);
});

export default app;