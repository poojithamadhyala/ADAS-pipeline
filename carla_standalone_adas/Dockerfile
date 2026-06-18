FROM python:3.10-slim

WORKDIR /app

RUN pip install --no-cache-dir carla==0.9.15

COPY carla_adas_standalone.py /app/carla_adas_standalone.py

CMD ["python", "carla_adas_standalone.py"]
