import { Pool } from 'pg';
import dotenv from 'dotenv';
import path from 'path';

// Load environment variables from the root of the auth-server
const envPath = path.resolve(process.cwd(), '.env');
console.log('Loading .env from:', envPath);
dotenv.config({ path: envPath });

const dbConfig = {
    connectionString: process.env.DATABASE_URL,
    ssl: {
        rejectUnauthorized: false
    },
    connectionTimeoutMillis: 5000 // Fail fast
};

const pool = new Pool(dbConfig);

async function checkConnection() {
    console.log('Testing database connection...');
    try {
        const client = await pool.connect();
        console.log('Successfully connected to database!');
        const res = await client.query('SELECT NOW()');
        console.log('Database time:', res.rows[0].now);
        client.release();
    } catch (error) {
        console.error('Connection failed:', error);
    } finally {
        await pool.end();
    }
}

checkConnection();
