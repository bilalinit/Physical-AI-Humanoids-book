import { auth } from '../../config/auth.js';
import { logAuthentication } from '../../middleware/logger.js';
import { Request, Response } from 'express';

export default async (req: Request, res: Response) => {
  try {
    // Get session info before signing out (if available)
    const session = await (auth.api as any).getSession({
      headers: req.headers as any
    });

    // Sign out user with Better Auth
    // Better Auth will handle session token internally from cookies/headers
    await (auth.api as any).signOut({
      headers: req.headers as any,
      body: {} as any // Better Auth will handle session token internally
    });

    // Log the signout operation
    logAuthentication(session?.user, 'signout', true);

    // Return success response
    res.status(200).json({
      success: true,
      message: 'Successfully signed out'
    });
  } catch (error: any) {
    console.error('Signout error:', error);

    res.status(500).json({
      error: 'Internal server error during signout',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
};