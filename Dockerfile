# Base image
FROM python:3.9-slim

# System dependencies for OpenGL and GUI support
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

# Working directory
WORKDIR /app

# Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Project source code
COPY . .

# Execution entry point
CMD ["python", "src/main.py"]