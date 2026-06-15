#!/usr/bin/env bash
# Provision a GitHub Actions self-hosted runner for gnc-sim: install the full toolchain
# (C++ + Emscripten + Python + Node), then download, register, and (optionally) install the runner
# as a service. Idempotent-ish; safe to re-run. See docs/SELF_HOSTED_RUNNER.md.
#
# Usage:
#   REPO=nbloem26/gnc-sim ./scripts/setup-self-hosted-runner.sh
# Registration token resolution order:
#   1) $RUNNER_TOKEN if set
#   2) `gh` CLI (if authenticated) -> fetches a fresh registration token automatically
#   3) prompt you to paste one from: repo Settings -> Actions -> Runners -> New self-hosted runner
set -euo pipefail

REPO="${REPO:-nbloem26/gnc-sim}"
RUNNER_LABELS="${RUNNER_LABELS:-self-hosted,linux,gnc-sim}"
RUNNER_DIR="${RUNNER_DIR:-$HOME/actions-runner-gnc-sim}"
EMSDK_VERSION="${EMSDK_VERSION:-3.1.61}"

log() { printf '\n\033[1;32m==>\033[0m %s\n' "$*"; }

# --- 1. Toolchain -----------------------------------------------------------------
log "Installing build toolchain (sudo apt)"
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  build-essential cmake ninja-build git curl ca-certificates jq \
  python3 python3-pip python3-venv

# --- Node.js (Ubuntu's nodejs+npm apt packages conflict; use NodeSource which bundles npm) ---
if ! command -v node >/dev/null 2>&1; then
  log "Installing Node.js 20 via NodeSource"
  curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
  sudo apt-get install -y nodejs
else
  log "Node.js already present ($(node -v)) - skipping"
fi

# --- 2. Emscripten SDK ------------------------------------------------------------
if [ ! -d "$HOME/emsdk" ]; then
  log "Installing Emscripten SDK ${EMSDK_VERSION} -> ~/emsdk"
  git clone https://github.com/emscripten-core/emsdk.git "$HOME/emsdk"
  (cd "$HOME/emsdk" && ./emsdk install "$EMSDK_VERSION" && ./emsdk activate "$EMSDK_VERSION")
else
  log "Emscripten already present at ~/emsdk (skipping)"
fi
# Make emsdk discoverable to the runner's jobs.
grep -q 'emsdk_env.sh' "$HOME/.bashrc" 2>/dev/null || \
  echo 'source "$HOME/emsdk/emsdk_env.sh" >/dev/null 2>&1 || true' >> "$HOME/.bashrc"

# --- 3. GitHub Actions runner -----------------------------------------------------
log "Fetching latest GitHub Actions runner"
mkdir -p "$RUNNER_DIR" && cd "$RUNNER_DIR"
RUNNER_VER="$(curl -fsSL https://api.github.com/repos/actions/runner/releases/latest | jq -r .tag_name | sed 's/^v//')"
TARBALL="actions-runner-linux-x64-${RUNNER_VER}.tar.gz"
if [ ! -f "$TARBALL" ]; then
  curl -fsSL -o "$TARBALL" \
    "https://github.com/actions/runner/releases/download/v${RUNNER_VER}/${TARBALL}"
  tar xzf "$TARBALL"
fi

# --- 4. Registration token --------------------------------------------------------
resolve_token() {
  if [ -n "${RUNNER_TOKEN:-}" ]; then echo "$RUNNER_TOKEN"; return; fi
  if command -v gh >/dev/null 2>&1 && gh auth status >/dev/null 2>&1; then
    gh api -X POST "repos/${REPO}/actions/runners/registration-token" -q .token
    return
  fi
  echo ""
}
TOKEN="$(resolve_token)"
if [ -z "$TOKEN" ]; then
  echo "No registration token. Get one from:"
  echo "  https://github.com/${REPO}/settings/actions/runners/new"
  read -r -p "Paste registration token: " TOKEN
fi

# --- 5. Configure + run -----------------------------------------------------------
log "Configuring runner for ${REPO} with labels: ${RUNNER_LABELS}"
./config.sh --unattended --url "https://github.com/${REPO}" --token "$TOKEN" \
  --labels "$RUNNER_LABELS" --name "${RUNNER_NAME:-$(hostname)-gnc-sim}" --replace

cat <<EOF

Runner configured in ${RUNNER_DIR}.
  Run interactively:   (cd ${RUNNER_DIR} && ./run.sh)
  Or install service:  (cd ${RUNNER_DIR} && sudo ./svc.sh install && sudo ./svc.sh start)

Security: this runner is for a PUBLIC repo. The self-hosted workflow intentionally has NO
pull_request trigger, so fork-PR code never runs here. Keep it that way. See docs/SELF_HOSTED_RUNNER.md.
EOF
