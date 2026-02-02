# GitHub Personal Access Token Setup

Use this guide to set up a Personal Access Token (PAT) so you can push to this repo over HTTPS.

## Step 1: Create a token on GitHub

1. Open: **https://github.com/settings/tokens**
2. Click **"Generate new token"** → **"Generate new token (classic)"**
3. Give it a name (e.g. `AAE4011-S22526 push`)
4. Choose an expiration (e.g. 90 days or "No expiration" for convenience)
5. Under **Scopes**, enable:
   - **repo** (full control of private repositories)
6. Click **"Generate token"**
7. **Copy the token immediately** — GitHub only shows it once. It looks like `ghp_xxxxxxxxxxxx`.

## Step 2: Configure Git to use the token

**Option A — Store credentials (recommended)**

After this, the first `git push` will ask for your credentials once; Git will then remember them.

```bash
cd /home/wws/catkin_ws/src/AAE4011-S22526

# Use the store credential helper (saves username + token to ~/.git-credentials)
git config credential.helper store

# Push; when prompted:
#   Username: your GitHub username (e.g. weisongwen)
#   Password: paste your token (not your GitHub password)
git push origin main
```

**Option B — Put token in remote URL (no prompt)**

Replace `YOUR_USERNAME` and `YOUR_TOKEN` with your GitHub username and the token you created.

```bash
git remote set-url origin https://YOUR_USERNAME:YOUR_TOKEN@github.com/weisongwen/AAE4011-S22526.git
git push origin main
```

⚠️ **Security:** Option B stores the token in `.git/config`. Do not commit this file or share it. Prefer Option A if others might see your config.

## Step 3: Verify

Run:

```bash
git push origin main
```

If it succeeds, your token is set up correctly.

## Troubleshooting

- **"Authentication failed"** — Double-check username and that you pasted the full token (starts with `ghp_`).
- **"Token expired"** — Create a new token at https://github.com/settings/tokens and run the Step 2 commands again with the new token.
