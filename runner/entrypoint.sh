#!/usr/bin/env bash
# Registers an EPHEMERAL self-hosted runner, runs one job, then exits (compose restarts it for the
# next job). Ephemeral = each job gets a clean runner, which is the safer pattern. Requires a PAT
# with `repo` scope (GH_PAT) to mint a short-lived registration token at startup.
set -euo pipefail

REPO="${REPO:-nbloem26/gnc-sim}"
LABELS="${RUNNER_LABELS:-self-hosted,linux,gnc-sim}"
NAME="${RUNNER_NAME:-docker-$(hostname)}"

if [ -z "${GH_PAT:-}" ]; then
  echo "error: GH_PAT (a GitHub PAT with 'repo' scope) is required to fetch a registration token." >&2
  exit 1
fi

# Mint a fresh registration token from the PAT (tokens expire in ~1h; fine for ephemeral).
TOKEN="$(curl -fsSL -X POST \
  -H "Authorization: Bearer ${GH_PAT}" \
  -H "Accept: application/vnd.github+json" \
  "https://api.github.com/repos/${REPO}/actions/runners/registration-token" | jq -r .token)"

cd /home/runner
./config.sh --unattended --ephemeral --replace \
  --url "https://github.com/${REPO}" --token "$TOKEN" \
  --labels "$LABELS" --name "$NAME"

# Deregister cleanly on stop.
cleanup() { ./config.sh remove --token "$TOKEN" || true; }
trap cleanup EXIT

./run.sh
