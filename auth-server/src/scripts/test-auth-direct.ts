import dotenv from 'dotenv';
import path from 'path';

// Load environment variables SAME WAY as check-db
const envPath = path.resolve(process.cwd(), '.env');
console.log('Loading .env from:', envPath);
dotenv.config({ path: envPath });

import { auth } from '../config/auth.js'; // Import valid config

async function testSignup() {
    console.log('Testing Better Auth Direct Signup...');
    try {
        const testEmail = `test-${Date.now()}@example.com`;
        console.log(`Attempting to sign up user: ${testEmail}`);

        const res = await auth.api.signUpEmail({
            body: {
                email: testEmail,
                password: 'Password123!',
                name: 'Test Agent User'
            }
        });

        console.log('Signup SUCCESS:', res);
    } catch (error) {
        console.log('Signup FAILED:', error);
        if (error instanceof Error) {
            console.log('Error name:', error.name);
            console.log('Error message:', error.message);
            console.log('Error stack:', error.stack);
            if ((error as any).errors) {
                console.log('Aggregate Errors:', (error as any).errors);
            }
        }
    } finally {
        process.exit(0);
    }
}

testSignup();
