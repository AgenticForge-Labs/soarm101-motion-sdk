FROM node:22-bookworm-slim

RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates python3 \
    && npm install -g @openai/codex \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 --shell /bin/bash sandbox \
    && mkdir -p /workspace /opt/agent-as-code \
    && chown -R sandbox:sandbox /workspace /home/sandbox

COPY AGENTS.md CLI.md /opt/agent-as-code/
COPY robotctl.py /usr/local/bin/robotctl
COPY openshell/codex/run-agent.sh /usr/local/bin/run-agent-as-code-codex
RUN chmod 0555 /usr/local/bin/robotctl /usr/local/bin/run-agent-as-code-codex

USER sandbox
WORKDIR /workspace
ENV HOME=/home/sandbox
