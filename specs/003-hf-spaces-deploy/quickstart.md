# Quickstart: Deploy RAG Chatbot to Hugging Face Spaces

**Estimated Time**: 15-20 minutes
**Prerequisites**: HF account, configured external services (Neon, Qdrant Cloud, Gemini API)

## Step 1: Create HF Space

1. Go to [huggingface.co/new-space](https://huggingface.co/new-space)
2. Enter Space name: `rag-chatbot` (or your preferred name)
3. Select **Docker** as the SDK
4. Choose visibility: Public or Private
5. Click **Create Space**

Your Space URL will be: `https://username-rag-chatbot.hf.space`

---

## Step 2: Configure Secrets

Navigate to **Settings** > **Repository Secrets** and add:

| Secret Name | Value | How to Get |
|-------------|-------|------------|
| `GEMINI_API_KEY` | Your Gemini API key | [Google AI Studio](https://aistudio.google.com/) |
| `QDRANT_URL` | `https://xxx.qdrant.io` | Qdrant Cloud dashboard |
| `QDRANT_API_KEY` | Your Qdrant API key | Qdrant Cloud dashboard |
| `QDRANT_COLLECTION` | `docusaurus-book` | Your collection name |
| `DATABASE_URL` | `postgresql://...?sslmode=require` | Neon dashboard |
| `ALLOWED_ORIGINS` | `https://your-app.vercel.app` | Your Vercel URL |
| `LOG_LEVEL` | `INFO` | - |

**Important**: Include `sslmode=require` in DATABASE_URL for Neon.

---

## Step 3: Upload Deployment Files

### Option A: Git Clone (Recommended)

```bash
# Clone your Space repository
git clone https://huggingface.co/spaces/YOUR_USERNAME/rag-chatbot
cd rag-chatbot

# Copy files from huggingface-spaces/ directory
cp -r /path/to/project/huggingface-spaces/* .

# Push to HF
git add .
git commit -m "Deploy RAG chatbot backend"
git push
```

### Option B: Web Upload

1. Go to your Space's **Files** tab
2. Click **Add file** > **Upload files**
3. Upload all files from the `huggingface-spaces/` directory:
   - `README.md`
   - `Dockerfile`
   - `requirements.txt`
   - `app/` folder (all files)

---

## Step 4: Monitor Build

1. Go to your Space's main page
2. Watch the **Logs** tab for build progress
3. Build typically takes 2-5 minutes

**Expected Log Output**:
```
Building Docker image...
Step 1/10 : FROM python:3.11-slim
...
Successfully built [image-id]
Starting container...
INFO:     Started server process
INFO:     Application startup complete
```

---

## Step 5: Verify Deployment

### Health Check

```bash
curl https://YOUR_USERNAME-rag-chatbot.hf.space/api/health
```

**Expected Response**:
```json
{
  "status": "healthy",
  "services": {
    "fastapi": "ok",
    "qdrant": {"status": "connected", "collection": "docusaurus-book"},
    "postgres": {"status": "connected"},
    "gemini": {"status": "ok"}
  },
  "timestamp": "2025-12-22T..."
}
```

### Test Chat Endpoint

```bash
curl -X POST https://YOUR_USERNAME-rag-chatbot.hf.space/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello", "mode": "general"}'
```

---

## Step 6: Update Frontend

### Environment Variable Configuration

The frontend needs to know where to send API requests. Update your Vercel environment variable:

| Variable | Format | Example |
|----------|--------|---------|
| `REACT_APP_API_BASE_URL` | `https://USERNAME-SPACENAME.hf.space` | `https://johndoe-rag-chatbot.hf.space` |

### Vercel Dashboard Steps

1. Go to [Vercel Dashboard](https://vercel.com/dashboard)
2. Select your project
3. Go to **Settings** > **Environment Variables**
4. Add or update:
   ```
   REACT_APP_API_BASE_URL=https://YOUR_USERNAME-rag-chatbot.hf.space
   ```
5. Click **Save**
6. **Redeploy** the frontend:
   - Go to **Deployments** tab
   - Click the **...** menu on the latest deployment
   - Select **Redeploy**
   - Wait for the new deployment to complete

**Important**: A redeploy is required after changing environment variables for the changes to take effect.

### Verify Frontend Connectivity

After redeploying, verify the frontend can communicate with the backend:

1. **Open Browser DevTools**: Press F12 or right-click > Inspect
2. **Go to Network tab**: Filter by "Fetch/XHR"
3. **Send a chat message**: Type a message in the chat widget
4. **Check for CORS errors**: Look for red error messages in Console tab
5. **Verify SSE connection**: Look for a request to `/api/chat` that stays open (pending status)

### Expected Behavior

- Chat messages should stream in token-by-token
- No CORS errors in the console
- Network tab shows successful `/api/chat` requests
- Source citations appear after the response completes

---

## Verification Checklist

- [ ] HF Space shows "Running" status
- [ ] `/api/health` returns HTTP 200
- [ ] All services show "connected" in health response
- [ ] Frontend can send chat messages
- [ ] SSE streaming works correctly
- [ ] No CORS errors in browser console

---

## Troubleshooting

### Build Fails

**Check**: Logs tab for specific error

**Common Issues and Fixes**:

| Issue | Cause | Fix |
|-------|-------|-----|
| Permission denied | Missing `--chown=user` | Ensure all COPY commands use `--chown=user` |
| Python not found | Wrong base image | Use `python:3.11-slim` |
| pip install fails | Missing system deps | Add `gcc g++ curl` to apt-get install |
| Module not found | requirements.txt issue | Verify all packages are listed |

**Example Fix for Permission Error**:
```dockerfile
# Wrong
COPY requirements.txt .

# Correct
COPY --chown=user requirements.txt .
```

### Health Returns 503

**Check**: Which service is disconnected in the response

**Service-Specific Troubleshooting**:

| Service | Common Cause | Fix |
|---------|--------------|-----|
| Qdrant | Invalid URL or API key | Verify URL includes `https://` and ends with `.qdrant.io` |
| Postgres | Missing sslmode | Add `?sslmode=require` to DATABASE_URL |
| Gemini | Invalid API key | Regenerate key at [AI Studio](https://aistudio.google.com/) |

**Debug Command**:
```bash
# Check detailed health status
curl -s https://YOUR_USERNAME-rag-chatbot.hf.space/api/health | jq .
```

### CORS Errors

**Symptoms**:
- Browser console shows: "Access to fetch at ... has been blocked by CORS policy"
- Chat requests fail but health endpoint works

**Fix**:
1. Go to HF Space Settings > Repository Secrets
2. Update `ALLOWED_ORIGINS` to include your frontend URL:
   ```
   ALLOWED_ORIGINS=https://your-app.vercel.app,https://username-rag-chatbot.hf.space
   ```
3. Restart the Space (Settings > Factory reboot)

**Verify CORS Headers**:
```bash
curl -I -X OPTIONS https://YOUR_USERNAME-rag-chatbot.hf.space/api/chat \
  -H "Origin: https://your-app.vercel.app" \
  -H "Access-Control-Request-Method: POST"
```

### Cold Start Takes Too Long

**Expected Behavior**:
- Free tier Spaces sleep after ~15 minutes of inactivity
- Wake-up time: 30-60 seconds
- During startup, `/api/health` returns 503 with `status: "starting"`

**Handling in Frontend**:
- Show a loading indicator during cold start
- Retry failed requests after a delay
- Consider upgrading to a paid tier for always-on

### SSE Streaming Not Working

**Symptoms**:
- Response arrives all at once instead of streaming
- Browser shows chunked transfer but no live updates

**Causes and Fixes**:
| Cause | Fix |
|-------|-----|
| Proxy buffering | Ensure no proxy is buffering responses |
| Missing headers | Verify `Cache-Control: no-cache` in response |
| Connection timeout | Check if firewall allows long-lived connections |

---

## Redeployment Process

### When to Redeploy

- Code changes in `huggingface-spaces/` directory
- Updated requirements.txt (new dependencies)
- Dockerfile modifications

### How to Redeploy

**Option A: Git Push (Recommended)**
```bash
# From your project directory
cd huggingface-spaces/

# Make changes...

# Push to HF Spaces
git add .
git commit -m "Update: description of changes"
git push
```

**Option B: Web Interface**
1. Go to your Space's **Files** tab
2. Upload modified files
3. Space will auto-rebuild

### Secrets Update (No Redeploy Needed)

Updating secrets in HF Settings takes effect immediately after Space restart:
1. Go to Settings > Repository Secrets
2. Update the secret value
3. Go to Settings > Factory reboot (or wait for auto-restart)

### Zero-Downtime Considerations

- HF Spaces does rolling deployments automatically
- Build errors will prevent deployment (current version stays live)
- Monitor Logs tab during deployment

---

## Next Steps

1. **Custom Domain**: Configure via HF Settings (Pro feature)
2. **GitHub Integration**: Connect repo for auto-deploy on push
3. **Monitoring**: Check HF Spaces logs for runtime errors
4. **Upgrade Tier**: Consider Pro for always-on and more resources
