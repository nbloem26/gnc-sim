# Self-hosted CI runner

Run the gnc-sim pipeline — and heavier Monte Carlo campaigns not worth cloud minutes — on your own
hardware via a GitHub Actions self-hosted runner.

## What runs where

| Workflow | Trigger | Runner | Purpose |
|---|---|---|---|
| `.github/workflows/ci.yml` | every PR + push | **github-hosted** `ubuntu-latest` | the gate for pull requests (fork-safe) |
| `.github/workflows/self-hosted.yml` | `workflow_dispatch` + push to `main` | **self-hosted** `[self-hosted, linux, gnc-sim]` | full pipeline + a **5000-case** Monte Carlo (`configs/montecarlo_heavy.json`) |

The PR gate stays on github-hosted runners. The self-hosted workflow is the only one that touches
your hardware, and it adds the large MC campaign the cloud CI skips.

## Security (read this — it's a public repo)

Self-hosted runners on public repos are dangerous *if* untrusted PRs can run on them. This setup is
built to prevent that:

- `self-hosted.yml` has **no `pull_request` trigger** — fork-PR code never executes on your box.
- Triggers are `workflow_dispatch` (you, manually) and `push` to `main` (already-merged code).
- A job guard `if: github.repository == 'nbloem26/gnc-sim'` stops repo *forks* from using it.
- Prefer **ephemeral** runners (the container path below) so each job gets a clean machine.
- In repo **Settings → Actions → General**, keep "Fork pull request workflows" restricted.

## Setup — pick one

### A. Script (bare metal / VM)
Installs the toolchain (C++, Emscripten, Python, Node), downloads the runner, and registers it:
```bash
REPO=nbloem26/gnc-sim ./scripts/setup-self-hosted-runner.sh
# then run it:
(cd ~/actions-runner-gnc-sim && ./run.sh)        # or: sudo ./svc.sh install && sudo ./svc.sh start
```
The token is fetched automatically if `gh` is authenticated; otherwise paste one from
**Settings → Actions → Runners → New self-hosted runner**.

### B. Container (recommended — ephemeral)
One command brings up a fully-provisioned ephemeral runner (fresh per job):
```bash
GH_PAT=<PAT with repo scope> docker compose -f runner/docker-compose.yml up --build
# N parallel runners:
GH_PAT=<...> docker compose -f runner/docker-compose.yml up --build --scale runner=4
```

### C. Manual
Follow GitHub's **Settings → Actions → Runners → New self-hosted runner**, then ensure the toolchain
from `scripts/setup-self-hosted-runner.sh` is present and `~/emsdk/emsdk_env.sh` is sourced for jobs.

## Labels
Jobs target `[self-hosted, linux, gnc-sim]`. Register runners with exactly these labels (the script
and container do this by default via `RUNNER_LABELS`).

## Trigger a run
- Push to `main`, or
- **Actions → Self-hosted CI (heavy) → Run workflow**.

The large Monte Carlo summary is uploaded as the `monte-carlo-heavy` artifact.

## Troubleshooting
- *Runner shows offline*: it only runs while `./run.sh`/the service/the container is up.
- *`emcmake: not found` in the WASM step*: the runner shell didn't source emsdk — the script adds it
  to `~/.bashrc`; the workflow also sources `$EMSDK`/`~/emsdk`/`/opt/emsdk` as a fallback.
- *Wrong labels*: re-register with `--labels self-hosted,linux,gnc-sim`.
