import express from 'express';
const router = express.Router();
import { auth } from '../../config/auth.js';

// Import specific auth routes
import signupRoute from './signup.js';
import signinRoute from './signin.js';
import signoutRoute from './signout.js';
import getSessionRoute from './get-session.js';

// Mount auth-specific routes
router.post('/signup', signupRoute);
router.post('/signin', signinRoute);
router.post('/signout', signoutRoute);
router.get('/get-session', getSessionRoute);

export default router;