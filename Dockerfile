# Build context is the PARENT directory (swarm-ops/) so the sibling contracts checkout
# is available until swarmops-contracts publishes a wheel:
#   docker build -f swarmops-fleet-sim/Dockerfile -t swarmops-fleet-sim ..
FROM python:3.12-slim

COPY --from=ghcr.io/astral-sh/uv:0.7.13 /uv /bin/uv
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy PYTHONUNBUFFERED=1

COPY swarmops-contracts/gen/python /src/swarmops-contracts/gen/python
COPY swarmops-fleet-sim /src/swarmops-fleet-sim
WORKDIR /src/swarmops-fleet-sim
RUN uv sync --no-dev

EXPOSE 8090
CMD ["/src/swarmops-fleet-sim/.venv/bin/fleet-sim"]
