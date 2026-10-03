FROM codex-v90-answerer:latest
RUN apt-get update && apt-get install -y --no-install-recommends iproute2 ppp && rm -rf /var/lib/apt/lists/*
COPY . /opt/v90/multilink/
RUN mkdir -p /opt/v90/wifi && chmod +x /opt/v90/multilink/*.py /opt/v90/multilink/start.sh
ENTRYPOINT ["/opt/v90/multilink/start.sh"]
