import { auth, pool } from '../../config/auth.js';
import { logUserRegistration } from '../../middleware/logger.js';
import { Request, Response } from 'express';

export default async (req: Request, res: Response) => {
  try {
    // Extract user data from request
    const { email, password, name, educationLevel, programmingExperience,
      roboticsBackground } = req.body;

    if (!email || !password || !name) {
      return res.status(400).json({
        error: 'Email, password, and name are required'
      });
    }

    // Validate learning preferences if provided
    const learningPreferences: any = {};
    if (educationLevel !== undefined) {
      const validEducationLevels = ['High School', 'Undergraduate', 'Graduate', 'Professional'];
      if (!validEducationLevels.includes(educationLevel)) {
        return res.status(400).json({
          error: 'Invalid education level. Must be one of: High School, Undergraduate, Graduate, Professional'
        });
      }
      learningPreferences.educationLevel = educationLevel;
    }

    if (programmingExperience !== undefined) {
      const validProgrammingExperiences = ['No Experience', 'Beginner', 'Intermediate', 'Advanced'];
      if (!validProgrammingExperiences.includes(programmingExperience)) {
        return res.status(400).json({
          error: 'Invalid programming experience. Must be one of: No Experience, Beginner, Intermediate, Advanced'
        });
      }
      learningPreferences.programmingExperience = programmingExperience;
    }

    if (roboticsBackground !== undefined) {
      const validRoboticsBackgrounds = ['No Experience', 'Hobbyist', 'Academic', 'Professional'];
      if (!validRoboticsBackgrounds.includes(roboticsBackground)) {
        return res.status(400).json({
          error: 'Invalid robotics background. Must be one of: No Experience, Hobbyist, Academic, Professional'
        });
      }
      learningPreferences.roboticsBackground = roboticsBackground;
    }

    // Create user with Better Auth
    const result = await (auth.api as any).signUpEmail({
      body: {
        email,
        password,
        name
      },
      userAgent: req.get('User-Agent'),
      ip: req.ip
    });

    // Get the user ID from the result
    const userId = result.user?.id || result.id;

    // Update user with custom learning preferences fields if any were provided
    if (Object.keys(learningPreferences).length > 0 && userId) {
      const updateFields: string[] = [];
      const updateValues: any[] = [];
      let paramIndex = 1;

      if (learningPreferences.educationLevel) {
        updateFields.push(`"educationLevel" = $${paramIndex++}`);
        updateValues.push(learningPreferences.educationLevel);
      }
      if (learningPreferences.programmingExperience) {
        updateFields.push(`"programmingExperience" = $${paramIndex++}`);
        updateValues.push(learningPreferences.programmingExperience);
      }
      if (learningPreferences.roboticsBackground) {
        updateFields.push(`"roboticsBackground" = $${paramIndex++}`);
        updateValues.push(learningPreferences.roboticsBackground);
      }

      if (updateFields.length > 0) {
        updateValues.push(userId);
        await pool.query(
          `UPDATE "user" SET ${updateFields.join(', ')} WHERE id = $${paramIndex}`,
          updateValues
        );
      }
    }

    // Fetch updated user to get all fields including custom ones
    const userResult = await pool.query(
      `SELECT id, email, name, "createdAt", "educationLevel", "programmingExperience", "roboticsBackground" FROM "user" WHERE id = $1`,
      [userId]
    );
    const user = userResult.rows[0];

    // Log user registration
    logUserRegistration(user, 'signup');

    // Return success response
    res.status(201).json({
      success: true,
      user: {
        id: user.id,
        email: user.email,
        name: user.name,
        createdAt: user.createdAt,
        educationLevel: user.educationLevel,
        programmingExperience: user.programmingExperience,

        roboticsBackground: user.roboticsBackground
      }
    });
  } catch (error: any) {
    console.error('Signup error:', error);

    // Handle specific Better Auth errors
    if (error.message && error.message.includes('User already exists')) {
      return res.status(409).json({
        error: 'User with this email already exists'
      });
    }

    res.status(500).json({
      error: 'Internal server error during signup',
      details: process.env.NODE_ENV === 'development' ? error.message : undefined
    });
  }
};