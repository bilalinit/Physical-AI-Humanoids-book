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

    // Fetch full user details from database to ensure custom fields are included
    // Better Auth session object might not have all custom fields by default
    const userResult = await auth.options.database.query(
      `SELECT * FROM "user" WHERE id = $1`,
      [user.id]
    );

    const fullUser = userResult.rows[0] || user;

    // Return session data
    res.status(200).json({
      success: true,
      session: {
        user: {
          id: fullUser.id,
          email: fullUser.email,
          name: fullUser.name,
          createdAt: fullUser.createdAt,
          educationLevel: fullUser.educationLevel,
          programmingExperience: fullUser.programmingExperience,
          softwareBackground: fullUser.softwareBackground,
          hardwareBackground: fullUser.hardwareBackground,
          roboticsBackground: fullUser.roboticsBackground
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