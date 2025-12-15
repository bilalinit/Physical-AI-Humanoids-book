import dotenv from 'dotenv';
dotenv.config();
import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

// Database configuration for Neon
// Database configuration for Neon
// Database configuration for Neon
const dbConfig = {
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  },
  max: 2, // Low max connection count for free tier stability
  idleTimeoutMillis: 30000,
  connectionTimeoutMillis: 10000
};

console.log(`Initializing database pool with URL: ${process.env.DATABASE_URL ? 'Set' : 'Missing'}`);

export const pool = new Pool(dbConfig);

// Better Auth configuration
export const authConfig = {
  database: pool,
  secret: process.env.BETTER_AUTH_SECRET as string,
  baseURL: process.env.BETTER_AUTH_URL || 'http://localhost:3001',
  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production
    password: {
      // Password validation rules
      minLength: 8,
      // Additional password requirements can be added here
    }
  },
  socialProviders: {
    // OAuth providers can be configured here
    // google: {
    //   clientId: process.env.GOOGLE_CLIENT_ID,
    //   clientSecret: process.env.GOOGLE_CLIENT_SECRET,
    // },
    // github: {
    //   clientId: process.env.GITHUB_CLIENT_ID,
    //   clientSecret: process.env.GITHUB_CLIENT_SECRET,
    // },
  },
  user: {
    // Additional user fields for learning preferences
    data: {
      educationLevel: {
        type: 'string',
        required: false,
        options: ['High School', 'Undergraduate', 'Graduate', 'Professional']
      },
      programmingExperience: {
        type: 'string',
        required: false,
        options: ['No Experience', 'Beginner', 'Intermediate', 'Advanced']
      },

      roboticsBackground: {
        type: 'string',
        required: false,
        options: ['No Experience', 'Hobbyist', 'Academic', 'Professional']
      }
    }
  },
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days in seconds
    updateAge: 24 * 60 * 60, // 24 hours in seconds
  },
  emailVerification: {
    sendOnSignUp: false, // Set to true in production
  },
  account: {
    accountLinking: {
      enabled: true,
      requireEmailVerification: false, // Set to true in production
    }
  },
  callbacks: {
    // Callbacks for user creation, updates, etc.
    sessionCreated: async (session: any) => {
      console.log('Session created for user:', session.user.id);
    },
    sessionRefreshed: async (session: any) => {
      console.log('Session refreshed for user:', session.user.id);
    },
    sessionDeleted: async (sessionId: string) => {
      console.log('Session deleted:', sessionId);
    }
  }
};

// Initialize Better Auth
export const auth = betterAuth(authConfig as any);