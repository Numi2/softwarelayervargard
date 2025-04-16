# Vargard Web UI (Next.js)

This directory contains a Next.js frontend for the Vargard dashboard.

Getting Started
---------------

1. Install dependencies:
   ```bash
   cd web
   npm install
   ```

2. Run development server:
   ```bash
   npm run dev
   ```

3. Open your browser at http://localhost:3000

The dashboard fetches telemetry and alerts from `/api/telemetry` and `/api/alerts`,
which can be proxied to the backend service running in Flask or FastAPI.