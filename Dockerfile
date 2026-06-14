# gnc-sim reproducible build environment (NOT a runtime server — Vercel serves the static web app).
# Provides the full Linux toolchain to build the native binary + tests, cross-compile to WASM via
# Emscripten, and run the Python post-processing.
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake ninja-build git ca-certificates curl \
      python3 python3-pip python3-venv \
      nodejs npm \
 && rm -rf /var/lib/apt/lists/*

# Emscripten SDK (pinned) for the WASM build.
ARG EMSDK_VERSION=3.1.61
RUN git clone https://github.com/emscripten-core/emsdk.git /opt/emsdk \
 && cd /opt/emsdk && ./emsdk install ${EMSDK_VERSION} && ./emsdk activate ${EMSDK_VERSION}
ENV PATH="/opt/emsdk:/opt/emsdk/upstream/emscripten:/opt/emsdk/node/*/bin:${PATH}"

# Python tooling for the post-processing/sensor rigor.
COPY postproc/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true

WORKDIR /workspace
CMD ["/bin/bash"]
