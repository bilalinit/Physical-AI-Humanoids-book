import express from 'express';
const router = express.Router();

// Import user profile routes
import meRoute from './me.js';

// Mount user profile routes
router.get('/me', meRoute);  // Get current user profile
router.put('/me', meRoute);  // Update current user profile

export default router;