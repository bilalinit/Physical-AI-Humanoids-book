import { auth } from '../../config/auth.js';
import { Request, Response } from 'express';

export default async (req: Request, res: Response) => {
  try {
    // Get current session
    const session = await (auth.api as any).getSession({
      headers: req.headers as any
    });

    if (!session) {
      return res.status(401).json({
        error: 'No active session found'
      });
    }

    // Handle GET request - return user profile
    if (req.method === 'GET') {
      const user = session.user as any;
      return res.status(200).json({
        success: true,
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
        }
      });
    }

    // Handle PUT request - update user profile
    if (req.method === 'PUT') {
      const { name, educationLevel, programmingExperience, softwareBackground, hardwareBackground, roboticsBackground } = req.body;

      // Update user profile
      const updatedUser = await (auth.api as any).updateUser({
        body: {
          userId: session.user.id,
          name: name || session.user.name,
          data: {
            educationLevel: educationLevel || (session.user as any).educationLevel,
            programmingExperience: programmingExperience || (session.user as any).programmingExperience,
            softwareBackground: softwareBackground || (session.user as any).softwareBackground,
            hardwareBackground: hardwareBackground || (session.user as any).hardwareBackground,
            roboticsBackground: roboticsBackground || (session.user as any).roboticsBackground
          }
        },
        headers: req.headers as any
      });

      return res.status(200).json({
        success: true,
        user: {
          id: updatedUser.id,
          email: updatedUser.email,
          name: updatedUser.name,
          createdAt: updatedUser.createdAt,
          educationLevel: updatedUser.educationLevel,
          programmingExperience: updatedUser.programmingExperience,
          softwareBackground: updatedUser.softwareBackground,
          hardwareBackground: updatedUser.hardwareBackground,
          roboticsBackground: updatedUser.roboticsBackground
        }
      });
    }

    // Method not allowed for other HTTP methods
    res.status(405).json({
      error: 'Method not allowed'
    });
  } catch (error: any) {
    console.error('User profile error:', error);

    res.status(500).json({
      error: 'Internal server error while retrieving/updating user profile',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
};