# Containerized GitHub Actions self-hosted runner for gnc-sim, preloaded with the full toolchain
# (C++ + Emscripten + Python + Node). Bring up with runner/docker-compose.yml. See
# docs/SELF_HOSTED_RUNNER.md. Runs EPHEMERAL by default (fresh runner per job = safer).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git curl ca-certificates jq sudo \
      python3 python3-pip python3-venv nodejs npm \
 && rm -rf /var/lib/apt/lists/*

# Emscripten SDK (pinned) for the WASM build + parity step.
ARG EMSDK_VERSION=3.1.61
RUN git clone https://github.com/emscripten-core/emsdk.git /opt/emsdk \
 && cd /opt/emsdk && ./emsdk install ${EMSDK_VERSION} && ./emsdk activate ${EMSDK_VERSION}
ENV EMSDK=/opt/emsdk
ENV PATH="/opt/emsdk:/opt/emsdk/upstream/emscripten:${PATH}"

# Non-root runner user (the Actions runner refuses to run as root).
RUN useradd -m runner && echo "runner ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER runner
WORKDIR /home/runner

# GitHub Actions runner.
ARG RUNNER_VERSION=2.317.0
RUN curl -fsSL -o runner.tar.gz \
      "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/actions-runner-linux-x64-${RUNNER_VERSION}.tar.gz" \
 && tar xzf runner.tar.gz && rm runner.tar.gz

COPY --chown=runner:runner entrypoint.sh /home/runner/entrypoint.sh
RUN chmod +x /home/runner/entrypoint.sh
ENTRYPOINT ["/home/runner/entrypoint.sh"]
