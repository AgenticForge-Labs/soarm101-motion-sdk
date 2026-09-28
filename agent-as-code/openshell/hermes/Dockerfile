FROM nousresearch/hermes-agent:latest

USER root
RUN mkdir -p /workspace /opt/agent-as-code /opt/data \
    && chown -R 10000:10000 /workspace /opt/data
COPY AGENTS.md CLI.md /opt/agent-as-code/
COPY robotctl.py /usr/local/bin/robotctl
COPY openshell/hermes/run-agent.sh /usr/local/bin/run-agent-as-code-hermes
RUN chmod 0555 /usr/local/bin/robotctl /usr/local/bin/run-agent-as-code-hermes

USER 10000:10000
WORKDIR /workspace
ENV HOME=/opt/data
ENV HERMES_HOME=/opt/data
ENV TERMINAL_ENV=local
