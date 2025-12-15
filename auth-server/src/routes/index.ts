import express, { Request, Response } from 'express';
const router = express.Router();

// Import auth routes
import authRoutes from './auth/index.js';
import userRoutes from './user/index.js';

// Mount auth routes
router.use('/auth', authRoutes);
router.use('/user', userRoutes);

// Health check endpoint
router.get('/health', (req: Request, res: Response) => {
  res.status(200).json({
    status: 'healthy',
    service: 'auth-server',
    timestamp: new Date().toISOString()
  });
});

// Root endpoint
router.get('/', (req: Request, res: Response) => {
  res.status(200).json({
    message: 'Better Auth Server for RAG Chatbot',
    version: '1.0.0',
    endpoints: {
      auth: '/auth',
      user: '/user',
      health: '/health'
    }
  });
});

export default router;