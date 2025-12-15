import cors, { CorsOptions } from 'cors';

// Determine allowed origins based on environment
const allowedOrigins: string[] = process.env.NODE_ENV === 'production'
  ? [
    process.env.FRONTEND_URL || 'https://physical-ai-humanoids-book-rag.netlify.app',
    'https://your-production-frontend.com'
  ]
  : [
    'http://localhost:3000', // Docusaurus frontend (local dev)
    'http://localhost:3001', // Auth server itself
    'http://localhost:8000', // Backend server
    'http://127.0.0.1:3000',
    'http://127.0.0.1:3001',
    'http://127.0.0.1:8000',
    'https://localhost:3000',
    'https://localhost:3001',
    'https://localhost:8000'
  ];

// CORS configuration
const corsOptions: CorsOptions = {
  origin: allowedOrigins,
  credentials: true, // Allow cookies and credentials
  methods: ['GET', 'POST', 'PUT', 'DELETE', 'OPTIONS', 'PATCH'],
  allowedHeaders: [
    'Origin',
    'X-Requested-With',
    'Content-Type',
    'Accept',
    'Authorization',
    'X-Auth-Token',
    'X-Requested-With'
  ],
  exposedHeaders: [
    'X-Total-Count',
    'X-Request-ID',
    'Set-Cookie',
    'Authorization'
  ]
};

// CORS middleware
const corsMiddleware = cors(corsOptions);

export { corsMiddleware, corsOptions, allowedOrigins };