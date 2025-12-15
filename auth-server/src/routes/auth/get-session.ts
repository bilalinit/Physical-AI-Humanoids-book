import { auth } from '../../config/auth.js';
import { logAuthentication } from '../../middleware/logger.js';
import { Request, Response } from 'express';

export default async (req: Request, res: Response) => {
  try {
    // Get session using Better Auth's built-in method
    const session = await (auth.api as any).getSession({
      headers: req.headers as any
    });

    if (!session) {
      // Log unauthenticated access attempt
      logAuthentication(null, 'get-session', false, { reason: 'no-active-session' });

      return res.status(401).json({
        error: 'No active session found'
      });
    }

    // Log successful session validation
    const user = session.user as any;
    logAuthentication(user, 'get-session', true);

    // Return session data
    res.status(200).json({
      success: true,
      session: {
        user: {
          id: user.id,
          email: user.email,
          name: user.name,
          createdAt: user.createdAt,
          educationLevel: user.educationLevel,
          programmingExperience: user.programmingExperience,
          softwareBackground: user.softwareBackground,
          hardwareBackground: user.hardwareBackground,
          roboticsBackground: user.roboticsBackground
        },
        expiresAt: session.expiresAt
      }
    });
  } catch (error: any) {
    // Log failed session validation
    logAuthentication(null, 'get-session', false, { error: error.message });

    console.error('Get session error:', error);

    res.status(500).json({
      error: 'Internal server error while retrieving session',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
};