import dotenv from 'dotenv';
import path from 'path';
import { betterAuth } from 'better-auth';
import { Pool } from 'pg';

const envPath = path.resolve(process.cwd(), '.env');
dotenv.config({ path: envPath });

const dbConfig = {
    connectionString: process.env.DATABASE_URL,
    ssl: {
        rejectUnauthorized: false
    },
    connectionTimeoutMillis: 5000
};

const pool = new Pool(dbConfig);
import { PostgresDialect } from 'kysely';

console.log('Testing Isolated Better Auth with Dialect...');

const auth = betterAuth({
    database: {
        dialect: new PostgresDialect({
            pool
        })
    },
    secret: process.env.BETTER_AUTH_SECRET || 'test-secret',
    emailAndPassword: { enabled: true }
});

async function testIsolated() {
    try {
        console.log('Connecting to pool...');
        await pool.connect(); // Verify pool explicitly first
        console.log('Pool connected.');

        const testEmail = `test-iso-${Date.now()}@example.com`;
        console.log(`Signing up: ${testEmail}`);

        const res = await (auth.api as any).signUpEmail({
            body: {
                email: testEmail,
                password: 'Password123!',
                name: 'Iso Test'
            }
        });
        console.log('Signup SUCCESS:', res);
    } catch (error) {
        console.log('Signup FAILED:', error);
        if (error instanceof Error) {
            console.log('Error message:', error.message);
            console.log('Error stack:', error.stack);
            console.log('Error toString:', error.toString());
        }
    } finally {
        process.exit(0);
    }
}

testIsolated();
