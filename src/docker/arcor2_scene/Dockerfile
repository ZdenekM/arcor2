FROM python:3.11.12-bookworm as deps
COPY src.python.arcor2_scene.scripts/scene.pex /binary.pex
RUN PEX_TOOLS=1 PYTHONOPTIMIZE=1 /usr/local/bin/python /binary.pex venv --scope=deps --compile /bin/app

FROM python:3.11.12-bookworm as srcs
COPY src.python.arcor2_scene.scripts/scene.pex /binary.pex
RUN PEX_TOOLS=1 PYTHONOPTIMIZE=1 /usr/local/bin/python /binary.pex venv --scope=srcs --compile /bin/app

FROM python:3.11.12-bookworm

# libgomp1 and libusb-1.0-0 are because of Open3D
# curl is for healthcheck
RUN apt-get update \
        && apt-get install -y -q --no-install-recommends libgl1-mesa-glx=22.3.6-1+deb12u1 libglib2.0-0=2.74.6-2+deb12u5 libgomp1=12.2.0-14 libusb-1.0-0=2:1.0.26-1 curl=7.88.1-10+deb12u12 \
        && apt-get clean \
        && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/app/pex"]
COPY --from=deps /bin/app /bin/app
COPY --from=srcs /bin/app /bin/app

HEALTHCHECK --interval=5s --start-period=60s CMD curl -f http://localhost:5013/healthz/ready || exit 1
