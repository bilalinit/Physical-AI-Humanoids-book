import { auth } from '../../config/auth.js';
import { logAuthentication } from '../../middleware/logger.js';
import { Request, Response } from 'express';

export default async (req: Request, res: Response) => {
  try {
    // Extract credentials from request
    const { email, password } = req.body;

    if (!email || !password) {
      return res.status(400).json({
        error: 'Email and password are required'
      });
    }

    // Sign in user with Better Auth
    const session = await (auth.api as any).signInEmail({
      body: {
        email,
        password
      },
      userAgent: req.get('User-Agent'),
      ip: req.ip
    });

    // Log successful authentication
    logAuthentication(session.user, 'signin', true);

    // Return success response with session
    res.status(200).json({
      success: true,
      session: {
        user: {
          id: session.user.id,
          email: session.user.email,
          name: session.user.name,
          createdAt: session.user.createdAt,
          educationLevel: session.user.educationLevel,
          programmingExperience: session.user.programmingExperience,
          softwareBackground: session.user.softwareBackground,
          hardwareBackground: session.user.hardwareBackground,
          roboticsBackground: session.user.roboticsBackground
        },
        // Include session token in response for client-side storage if needed
        // In most cases, Better Auth handles this via cookies
      }
    });
  } catch (error: any) {
    // Log failed authentication
    logAuthentication(null, 'signin', false, { error: error.message });

    console.error('Signin error:', error);

    // Handle specific Better Auth errors
    if (error.message && error.message.includes('Invalid credentials')) {
      return res.status(401).json({
        error: 'Invalid email or password'
      });
    }

    res.status(500).json({
      error: 'Internal server error during signin',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
};