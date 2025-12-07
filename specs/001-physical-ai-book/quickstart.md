# Quickstart Guide: Docusaurus Book for Physical AI & Humanoid Robotics

## Prerequisites

- Node.js 20+ (for Docusaurus frontend)
- Python 3.12+ (for backend services)
- Docker and Docker Compose (for Qdrant vector database)
- Git
- NVIDIA GPU drivers (if using GPU acceleration)

## Local Development Setup

### 1. Clone and Navigate to Repository
```bash
git clone <repository-url>
cd hackathon-book
```

### 2. Set up Environment Files
Copy the example environment files and configure your settings:

```bash
# For the main project root
cp .env.example .env

# For the backend service
cp backend/.env.example backend/.env

# For the auth server
cp auth-server/.env.example auth-server/.env
```

### 3. Install Dependencies

#### Frontend (Docusaurus)
```bash
cd frontend
npm install
```

#### Backend (FastAPI)
```bash
cd backend
pip install -r requirements.txt
# Or using uv if available
uv pip install -r requirements.txt
```

#### Auth Server (Node.js)
```bash
cd auth-server
npm install
```

### 4. Start Services

#### Option A: All Services Separately (for development)
Terminal 1 - Frontend (Docusaurus):
```bash
cd frontend
npm start
# Serves on http://localhost:3000
```

Terminal 2 - Auth Server:
```bash
cd auth-server
npm run dev
# Serves on http://localhost:3001
```

Terminal 3 - Backend:
```bash
cd backend
python -m uvicorn main:app --reload --port 8000
# Serves on http://localhost:8000
```

#### Option B: Using Docker Compose (for production-like environment)
```bash
docker-compose up -d
```

### 5. Populate Documentation Content

To populate the Docusaurus site with the 12 chapters of the Physical AI book:

```bash
# Navigate to the documentation directory
cd frontend/docs

# The documentation files are already structured according to the spec:
# ├── intro.md
# ├── part-i-infrastructure/
# │   ├── index.md
# │   ├── chapter-1-hardware-os-config.md
# │   └── chapter-2-edge-ecosystem.md
# ├── part-ii-ros-nervous-system/
# │   ├── index.md
# │   ├── chapter-3-ros2-architecture.md
# │   └── chapter-4-urdf-kinematics.md
# ... and so on for all 12 chapters
```

### 6. Initialize Vector Database (for RAG functionality)

After adding documentation content, initialize the Qdrant vector database:

```bash
cd backend
python ingest.py
```

This will process all Markdown files in the `docs/` directory and store them in the vector database for AI-powered search and chat.

### 7. Configure Docusaurus

The Docusaurus configuration is in `frontend/docusaurus.config.ts` and includes:

- Site metadata (title, tagline, URL)
- Theme configuration
- Plugin configurations (search, sitemap, etc.)
- Navigation sidebar (generated from `sidebars.ts`)
- Mermaid diagram support
- Code block syntax highlighting

### 8. Development Workflow

#### Adding New Documentation Content
1. Create a new Markdown file in the appropriate part directory
2. Add it to the sidebar configuration in `sidebars.ts`
3. Run the ingestion script if you want it to be searchable by the AI:

```bash
cd backend
python ingest.py
```

#### Running Tests
```bash
# Frontend tests
cd frontend
npm test

# Backend tests
cd backend
pytest

# Auth server tests
cd auth-server
npm test
```

#### Building for Production
```bash
cd frontend
npm run build
```

## API Endpoints

Once all services are running, you can access:

- **Frontend (Docusaurus)**: http://localhost:3000
- **Auth Server**: http://localhost:3001
- **Backend API**: http://localhost:8000
- **Backend API Docs**: http://localhost:8000/docs

## Troubleshooting

### Common Issues

1. **Port conflicts**: Ensure ports 3000, 3001, and 8000 are available
2. **Missing environment variables**: Verify all `.env` files are properly configured
3. **Python dependency issues**: Use `uv` or ensure Python 3.12+ is installed
4. **Node version issues**: Ensure Node.js 20+ is installed

### Verifying Setup

To verify your setup is working correctly:

1. Visit http://localhost:3000 - You should see the Docusaurus documentation site
2. Visit http://localhost:8000/docs - You should see the FastAPI documentation
3. Test the search functionality on the frontend
4. Verify that the AI chat feature can be accessed (requires backend and auth server)